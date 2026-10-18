//! Cooperative, period-based scheduler of the robomorph stack.
//!
//! Three concepts live here:
//!
//! * `Process`: the unit of scheduling. A process owns a period (set with
//!   `set_period_from_freq`) and a main task, `exec`, that its processes pipeline calls once the
//!   period has elapsed. Processes exchange data in two ways: synchronously through the *pipe* (the
//!   `Option<AnyMessage>` returned by `exec` is handed as `input` to the next process of the same
//!   processes pipeline in the same pass) and asynchronously through tokio `broadcast` channels
//!   (`set_sender` / `set_receiver`).
//! * `ProcessesPipeline`: an ordered list of processes sharing one thread and one clock.
//!   `run_once` executes every process whose due instant has passed, in registration order, then
//!   sleeps until the nearest due instant.
//! * `Scheduler`: owns the *main pipeline* (run on the caller's thread by calling
//!   `run_main_pipeline()` in a `loop`, as the Gazebo binary does), the *side pipelines* (each run
//!   on a dedicated thread by `start_all_side_pipelines`) and the `HardwareInterface`s (started once
//!   by `start_all_interfaces`, which calls their `connect()`; they are NOT scheduled processes,
//!   they run their own RX / TX threads).
//!
//! Vocabulary: a *processes pipeline* is this execution unit of the scheduler. It is not a
//! *robotics pipeline* (a functional domain such as motor control, `docs/docs/pipelines/`), which
//! runs on one or more processes pipelines.
//!
//! Design rule learned in practice: a producer and its consumer must live in the SAME processes
//! pipeline (same clock), registered in order. Two pipelines at the same nominal frequency run on
//! different threads with independent clocks, drift in phase, and the consumer alternately sees
//! empty and doubled ticks (a beat between the two clocks). The Gazebo binary therefore registers
//! the vehicle controller, the remote control, the attitude controller and the motor mixer in the
//! main pipeline, all at the same frequency.

use std::{sync::{Arc, atomic::{AtomicBool, Ordering}, mpsc}, thread};
use std::time::{Duration, Instant};

use tokio::sync::broadcast::{Receiver, Sender, channel};

use crate::{communications::interface::HardwareInterface, messages::registered_message::AnyMessage};

/// Trait that give the fundamental methods of all the processes
///
/// A process is the unit of scheduling: a `ProcessesPipeline` calls `exec` at the process's own
/// period (`get_period`). Concrete processes of the stack: the vehicle controller (Gazebo side),
/// `XboxPadControl`, `AttitudeController` and `MotorsMixer`.
///
/// Two data paths exist between processes:
/// * the **pipe**: the value returned by `exec` is passed as `input` to the next process
///   registered in the same processes pipeline, within the same pass (see
///   `ProcessesPipeline::run_once`);
/// * **broadcast channels**: `set_sender` / `set_receiver` hand the process one end of a tokio
///   `broadcast` channel; a receiver is drained with `try_recv` at each `exec`.
///
/// Convention followed by the motion processes (`AttitudeController`, `MotorsMixer`): if a sender
/// is set AND currently has subscribers (`receiver_count() > 0`) the result is sent on the channel
/// and `exec` returns `None`; otherwise the result is returned on the pipe.
pub trait Process {
    /// Human readable name of the process, used in log prints only.
    fn set_name(&mut self, name: String);
    /// Execute the process task. The main task of the process
    ///
    /// `input` is the pipe: the value returned by the PREVIOUS process of the same processes
    /// pipeline in the same pass (`None` for the first process of the pass, when the previous one
    /// abstained, or when the previous one was not executed in this pass). Returning `Some` feeds the
    /// next process; returning `None` means "nothing to hand over".
    ///
    /// `dt` is the time elapsed since the previous START of this process's task, measured by its
    /// processes pipeline: about one period in steady state, more after a skipped tick. At the first
    /// execution there is no previous start, and `dt` is the nominal period (zero for a zero-period
    /// process).
    fn exec(&mut self, input: &Option<AnyMessage>, dt: Duration) -> Option<AnyMessage>;
    /// Helper function that force the Process to have receiver object to receive cmd from other process or interface
    ///
    /// The receiver is a subscription to a tokio `broadcast` channel: the asynchronous input of the
    /// process (e.g. the telemetry `Pose` for the attitude controller, the motor commands for the
    /// vehicle controller). Processes without such an input just log a warning.
    fn set_receiver(&mut self, receiver: Receiver<AnyMessage>);
    /// Helper function that force the Process to have sender object to send data to other process or interface
    ///
    /// The sending end of a tokio `broadcast` channel: the asynchronous output of the process. A
    /// `Sender` can be cloned, so several processes may publish on the same channel.
    fn set_sender(&mut self, sender: Sender<AnyMessage>);
    /// Set the execution period of the main task from a frequency in Hz.
    ///
    /// Every implementation computes `Duration::from_nanos(1_000_000_000 / frequency)` (integer
    /// division: 3 Hz gives 333_333_333 ns; `0` panics with a division by zero).
    fn set_period_from_freq(&mut self, frequency: u64);
    /// Execution period of the main task, as set by `set_period_from_freq`. Zero if it was never
    /// set: the process then has no grid of its own and FOLLOWS its predecessor in the processes
    /// pipeline (it runs in every pass where its predecessor ran); a zero-period FIRST process runs
    /// once per `run_once` call, in the first pass of the call (see `ProcessesPipeline::run_once`,
    /// step 5).
    fn get_period(&self) -> Duration;
    /// Name given by `set_name`.
    fn get_name(&self) ->String;
}

/// Struct that represent a Processes Pipeline
///
/// This processes pipeline runs the main tasks of the processes one after another in the order in which processes are registered in the pipeline.
///
/// All the processes of a processes pipeline share one thread and one clock: the pipeline is the
/// unit of synchronisation of the stack. The pipe (`Process::exec` input / output) only exists
/// inside a processes pipeline, between consecutive processes executed in the same pass.

//? before Sized because generic params is ALWAYS Sized
//  but we want to use dyn Trait that is not Sized
//  ? before Sized allow to put Sized or not Sized type / trait as we want
pub struct ProcessesPipeline<P: ?Sized = dyn Process> {
    /// Id of the processes pipeline
    pub pipeline_id: u8,
    /// List of all the processes and the execution frequency ot their main task
    ///
    /// The first process registered in the list is executed first, the second after and so on
    ///
    /// The `Instant` is the next due instant of the process: the registration time at first, then
    /// advanced by WHOLE periods after each execution until it is in the future (the grid: a late
    /// process skips the ticks it missed and keeps its phase). For a zero-period process: its
    /// predecessor's next due instant, or the end of the current call for the first process.
    ///
    /// The `Option<Instant>` is the instant at which the process last STARTED its task, `None`
    /// before its first execution: the measured `dt` handed to `exec` is computed from it. Both
    /// instants live here, next to each other, so that the processes themselves carry no
    /// scheduling state.
    processes: Vec<(Box<P>, Instant, Option<Instant>)>,
}

impl<P: Process + ?Sized> ProcessesPipeline<P> {
    /// Constructor
    pub fn new(pipeline_id: u8) -> Self {
        return Self { pipeline_id, processes: vec![]};
    }

    /// Add process in the list of all the processes
    ///
    /// Arguments:
    ///
    /// process:
    ///
    /// The process to add and execute in the future
    ///
    /// frequency_hz:
    ///
    /// The execution frequency of the process main task
    ///
    /// The process is registered as due immediately (`Instant::now()`).
    // NOTE: the `frequency_hz` argument described above no longer exists: the period is read from
    // the process itself (`Process::get_period`), so `set_period_from_freq` must have been called
    // on it before it is added.
    pub fn add_process(&mut self, process: Box<P>) {
        self.processes.push((process, Instant::now(), None));
    }

    /// Run the processes pipeline one time
    ///
    /// Run processes one time / several times according to thir frequencies
    ///
    /// Algorithm:
    /// 1. `end_instant` is computed at entry as the latest `(due instant + period)` over all the
    ///    processes (and never earlier than now).
    /// 2. The `while` loop makes *passes* over the processes, in registration order. A pass always
    ///    runs to its end: `end_instant` is tested AFTER the pass, never between two processes, so a
    ///    producer and the consumer registered right after it are never split into two passes.
    /// 3. In a pass, each process that is due is executed. Its output is the pipe (`input_state`),
    ///    handed to the NEXT process only: the pipe is emptied at the start of every pass and when a
    ///    process is skipped, so a process only ever receives what its immediate predecessor
    ///    produced in the same pass. Its `dt` is the measured time since its previous start (the
    ///    nominal period at its first execution).
    /// 4. After its execution, a process's due instant advances by whole periods until it is in the
    ///    future (the grid): a late process skips the ticks it missed and keeps its phase.
    /// 5. A zero-period process has no grid: it follows its predecessor (it runs when the predecessor
    ///    ran in this pass). A zero-period FIRST process runs once per call, in the first pass of the
    ///    call (a pass counter decides, not its due instant), and never makes the pipeline spin.
    /// 6. Between two passes the thread sleeps until the nearest due instant.
    ///
    /// In steady state, with every process at the same period P, a call makes one pass where nothing
    /// is due, then two passes where all the processes run (one per tick): a call lasts about 2 P,
    /// and every pass carries the pipe intact from the first process to the last.
    // History (2026-10-13): until then the pipe was declared per pass, `end_instant` was tested
    // BEFORE each process (a pass could be cut in the middle) and a late process was re-phased on
    // "now". A ~1 ms hiccup shifted the mixer's due instant behind the others; it then set
    // `end_instant` itself, the pass was cut right between the attitude stage and the mixer, and the
    // wrench was lost at every tick until another hiccup reshuffled the phases: the attitude loop
    // stayed open for up to 1.8 s in Gazebo. Steps 2 to 4 above are the fix.
    // NOTE: (b) with zero processes the `for` loop never runs and the call returns at once (the
    // `end_instant` test passes immediately, nothing to sleep for): a caller that loops on an empty
    // processes pipeline (a side pipeline thread) spins and burns a whole core.
    fn run_once(&mut self) {
        let len = self.processes.len();
        let start_instant= Instant::now();
        let mut end_instant= start_instant;
        //Compute the ending instant of the processes pipeline
        for i in 0..len {
            let (proc, instant, _)= &self.processes[i];
            if let Some(next_instant)= instant.checked_add(proc.get_period()) {
                end_instant= Instant::max(end_instant, next_instant);
            }
        }
        let mut run_finished= false;
        // true during the first pass of this call only: a zero-period FIRST process runs in it
        let mut first_pass= true;
        //loop until all processes have been executed
        while run_finished == false {
            let mut next_instant= end_instant;
            // the pipe of THIS pass: emptied at the start of every pass, so the first process never
            // receives what the last process of the previous pass returned
            let mut input_state= None;
            // did the predecessor (process i - 1) execute in this pass? A zero-period process follows it
            let mut predecessor_ran= false;
            //check what processes needs to be executed
            //multiple processes can be executed in a same execution loop
            for i in 0..len {
                // 1. get the instant of the previous process
                //      None if the is the first in the pipeline
                let mut previous_instant= None;
                if i > 0 { 
                    previous_instant= Some(self.processes[i-1].1);
                }
                //get the process it will check for it's processe's task execution 
                let (proc, instant, last_start)= &mut self.processes[i];
                //check if the process is an event process (process with no period)
                let zero_period= proc.get_period() == Duration::from_millis(0);
                //check if the processe's task execution is due, needs to be executed
                let is_due;
                //IF the event process is inside the pipeline(means it is not the starting process)
                //it follows its predecessor: it is due only if its predecessor ran in this pass,
                //whatever its own instant says
                if zero_period && i > 0 {
                    is_due= predecessor_ran;
                }
                //ELSE IF it's the starting event process (first of the pipeline, period 0): it runs once
                //per run_once call, in the FIRST pass only. Its own instant cannot decide: it would be
                //due again in the last pass of the call, then in the first pass of the next call, i.e.
                //two runs 0.2 ms apart, then nothing for a whole call
                else if zero_period {
                    is_due= first_pass;
                }
                //ELSE it's a timed process: its own instant decides
                else {
                    is_due= instant.elapsed() > Duration::from_millis(0);
                }
                //IF it's time to execute the process
                if is_due {
                    //measured dt: the time since this process last STARTED its task (its previous
                    //execution), the nominal period at its first execution. Not `instant.elapsed()`:
                    //`instant` is the due instant of THIS execution, so that would be how late the
                    //process starts within its pass (~1.5 ms for the mixer), not the ~10 ms elapsed
                    let now= Instant::now();
                    let dt= match last_start {
                        Some(previous_start) => now.duration_since(*previous_start),
                        None => proc.get_period(),
                    };
                    *last_start= Some(now);
                    //execute the process task with the measured dt
                    input_state= proc.exec(&input_state, dt);
                    //IF it's a timed process
                    //compute the next task execution
                    if zero_period == false {
                        let mut is_future_instant= false;
                        while is_future_instant == false {
                            if let Some(nxt_instant)= instant.checked_add(proc.get_period()) {
                                //IF the next process instant is in the future, means Instant::now() is earlier
                                if nxt_instant.elapsed() == Duration::from_millis(0) {
                                    //the next process execution instant is find
                                    is_future_instant= true;
                                }
                                *instant= nxt_instant;
                            } else {
                                // Instant overflow (never in practice): stop instead of looping forever
                                is_future_instant= true;
                            }
                        }
                    }
                }
                //IF it's not the time to execute process task, throw the input state
                //to not process wrong state for the next process that will execute its task
                else {
                    input_state= None;
                }
                //IF it's a event process
                //save the next execution instant
                if zero_period {
                    //end_instant only for the first process: first_pass decides when it runs, the
                    //instant only has to stay in the future so that the sleep below is not cut short
                    *instant= end_instant;
                    //previous_process_instant if the current process is inside the pipeline(not the starting process)
                    if let Some(prev_instant) = previous_instant {
                        *instant= prev_instant;
                    }
                }
                //hand this execution status to the NEXT process of the pass: it becomes its predecessor_ran
                //(an event process placed right after this one runs only if this one ran in this pass)
                predecessor_ran= is_due;
                next_instant= Instant::min(*instant, next_instant);
            }
            //the next passes of this call are not the first one any more
            first_pass= false;
            //IF the ending instant of the processes pipeline has been reached
            let elapsed_time= end_instant.elapsed();
            if elapsed_time > Duration::from_millis(0) {
                run_finished= true;
                break;
            }
            //IF the processes executions loop is finished but the ending process pipeline's instant is not reached
            if run_finished == false {
                //wait until the very next process execution's instant
                thread::sleep(next_instant.saturating_duration_since(Instant::now()));
            }
        }
    }
}

/// Struct that represent the scheduler
///
/// The goal of the scheduler is to manage the processes pipelines
///
/// Life cycle used by the Gazebo binary: register processes / interfaces, `start_all_interfaces()`,
/// `start_all_side_pipelines()`, then `loop { run_main_pipeline() }` on the calling thread.
/// Interfaces are only started (their `connect()`), never scheduled: they run their own threads.
pub struct Scheduler {
    /// The processes pipeline executed in the main thread (same as the scheduler)
    ///
    /// Pipeline id 0, run on the caller's thread by `run_main_pipeline`.
    main_pipeline: ProcessesPipeline,
    /// The list of all the other processes pipelines that will run on other threads
    ///
    /// Indexed by pipeline id. Drained by `start_all_side_pipelines` (each pipeline is moved into
    /// its thread) and refilled by `stop_all_side_pipelines` (each thread returns its pipeline when
    /// it exits).
    side_pipelines: Vec<ProcessesPipeline<dyn Process + Send>>,
    /// The required objects to run / stop side pipelines that lived in dedicated threads
    ///
    /// One `(join handle, running flag)` per started side pipeline. The flag is polled by the thread
    /// between two `run_once` calls; the handle returns the pipeline when the thread exits.
    side_pipeline_handles: Vec<(std::thread::JoinHandle<ProcessesPipeline<dyn Process + Send>>, Arc<AtomicBool>)>,
    /// The list of the interfaces used to communicated with other external softwares / devices
    ///
    /// Only their `connect()` / `disconnect()` are driven from here.
    interfaces: Vec<Box<dyn HardwareInterface>>,
}

impl Scheduler {
    /// Constructor
    pub fn new() -> Self {
        return Self {
            main_pipeline: ProcessesPipeline::new(0),
            side_pipelines: vec![],
            side_pipeline_handles: vec![],
            interfaces: vec![],
        };
    }

    /// Register process in the main pipeline
    ///
    /// Arguments:
    ///
    /// process:
    ///
    /// The process to register
    ///
    /// frequency_hz:
    ///
    /// The process main task execution frequency
    // NOTE: the `frequency_hz` argument described above no longer exists: the period is read from
    // the process itself (`Process::get_period`), so `set_period_from_freq` must be called first.
    pub fn register_process(&mut self, process: Box<dyn Process>) {
        self.main_pipeline.add_process(process);
    }

    /// Register process in side pipeline
    ///
    /// Arguments:
    ///
    /// process:
    ///
    /// The process to register
    ///
    /// frequency_hz:
    ///
    /// The process main task execution frequency
    ///
    /// pipeline_id:
    ///
    /// ID of the processes pipeline in which the process is to be recorded
    ///
    /// The pipeline is created on first use; pipeline ids are indices into `side_pipelines`.
    // NOTE: the `frequency_hz` argument described above no longer exists (see `register_process`).
    // NOTE: pipeline ids must be dense: registering on id 2 first creates the empty pipelines 0 and
    // 1, and `start_all_side_pipelines` spawns a thread for each of them, which busy-loops on its
    // empty pipeline and burns a core (see `ProcessesPipeline::run_once`, NOTE (b)).
    pub fn register_side_process(&mut self, process: Box<dyn Process + Send>, pipeline_id: u8) {
        while self.side_pipelines.len() <= pipeline_id as usize {
            self.side_pipelines.push(ProcessesPipeline::new(pipeline_id));
        }
        self.side_pipelines[pipeline_id as usize].add_process(process);
    }

    /// Register interface
    ///
    /// Arguments:
    ///
    /// interface:
    ///
    /// The hardware interface to register
    pub fn register_interface(&mut self, interf: Box<dyn HardwareInterface>) {
        self.interfaces.push(interf);
    }

    /// Run main pipeline one time
    ///
    /// Notes:
    ///
    /// The scheduler must be to be mutable because the main pipeline also must be to be mutable in order to run.
    ///
    /// Meant to be called in a `loop` by the binary: each call is one `ProcessesPipeline::run_once`,
    /// which blocks (sleeping between passes) for roughly one period of the pipeline.
    pub fn run_main_pipeline(&mut self) {
        self.main_pipeline.run_once();
    }

    /// Run side pipelines in dedicated threads
    ///
    /// Notes:
    ///
    /// The scheduler must be to be mutable because it create and stored dedicated threads for all the side pipelines
    ///
    /// Each side pipeline is moved into its own thread, which loops on `run_once` while the running
    /// flag is set. The main pipeline is not concerned: it is run by `run_main_pipeline`.
    pub fn start_all_side_pipelines(&mut self) {
        for pipeline in self.side_pipelines.drain(..) {
            let running = Arc::new(AtomicBool::new(true));
            let running_clone = Arc::clone(&running);
            let handle = std::thread::spawn(move || {
                let mut pipeline = pipeline;
                while running_clone.load(Ordering::Relaxed) {
                    pipeline.run_once();
                }
                return pipeline;
            });

            self.side_pipeline_handles.push((handle, running));
        }
    }

    /// Start all the interfaces by trying to connect them to the external device / endpoint
    ///
    /// Notes:
    ///
    /// The scheduler must be mutable because running the connect() routine of an interface modifies that interface.
    pub fn start_all_interfaces(&mut self) {
        for interf in &mut self.interfaces {
            if let Err(e) = interf.connect() {
                eprintln!("Interface connection failed: {e}");
            }
        }
    }

    /// Stop all the side pipelines
    ///
    /// Notes:
    ///
    /// The scheduler must be mutable because each pipeline are returned by their dedicated thread
    ///
    /// And we needs to store them back to the list of side pipelines before restart dedicated threads again
    ///
    /// The running flag is only checked between two `run_once` calls, so each join waits for the
    /// current call to complete.
    // NOTE: an empty side pipeline returns from `run_once` at once (see NOTE (b) there): its thread
    // busy-loops until it is stopped, but the join itself completes.
    pub fn stop_all_side_pipelines(&mut self) {
        for (_, running) in &self.side_pipeline_handles {
            running.store(false, Ordering::Relaxed);
        }
        for (handle, _) in self.side_pipeline_handles.drain(..) {
            if let Ok(pipeline) = handle.join() {
                self.side_pipelines.push(pipeline);
            }
        }
    }

    /// Stop all the interfaces
    ///
    /// Notes:
    ///
    /// Scheduler must be mutable because unning the disconnect() routine of an interface modifies that interface.
    pub fn stop_all_interfaces(&mut self) {
        for interf in &mut self.interfaces {
            interf.disconnect();
        }
    }
}
