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
//! *robotics pipeline* (a functional domain such as motor control, `docs/src/pipelines/`), which
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
    /// pipeline in the same pass (`None` for the first process of the pass, or when the previous one
    /// abstained). Returning `Some` feeds the next process; returning `None` means "nothing to hand
    /// over".
    ///
    /// `dt` is meant to be the time elapsed since the previous execution, for integration.
    // NOTE: `ProcessesPipeline::run_once` currently passes the process PERIOD as `dt`, not the
    // measured elapsed time, so a late execution integrates with a step shorter than reality.
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
    /// Execution period of the main task, as set by `set_period_from_freq` (zero if it was never
    /// set, in which case the processes pipeline runs the process at every pass).
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
    /// advanced by one period after each execution (or reset to "now" when the process is late).
    processes: Vec<(Box<P>, Instant)>,
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
        self.processes.push((process, Instant::now()));
    }

    /// Run the processes pipeline one time
    ///
    /// Run processes one time / several times according to thir frequencies
    ///
    /// Algorithm:
    /// 1. `end_instant` is computed at entry as the latest `(due instant + period)` over all the
    ///    processes (and never earlier than now).
    /// 2. The `while` loop makes *passes* over the processes, in registration order, until
    ///    `end_instant` is passed. In a pass, each process whose due instant has elapsed is executed
    ///    and its due instant advanced by one period; the value it returns is the pipe
    ///    (`input_state`) handed to the next process of the same pass.
    /// 3. Between two passes the thread sleeps until the nearest due instant.
    ///
    /// In steady state, with every process at the same period P, one call performs one pass where
    /// all the processes run (pipe intact), sleeps about P, then exits on the following pass.
    // NOTE: (a) `input_state` is declared inside the `while` loop, so a value produced in one pass
    // is lost if its consumer only becomes due in the next pass. The pipe is therefore only reliable
    // when producer and consumer run in the same pass: same period, registered in order, same
    // processes pipeline. The `break` on `end_instant` can also cut a pass in the middle, in which
    // case the remaining processes run at the next call, with an empty pipe.
    // NOTE: (b) with zero processes the `for` loop never runs, `run_finished` is never set and the
    // `while` loop spins forever (sleeping 0 ns): an empty processes pipeline burns a whole core.
    // NOTE: (c) when a process is late (its next due instant is already in the past once it has
    // executed) its due instant is reset to `now`, which shifts its phase relative to the other
    // processes of the pipeline instead of catching up.
    fn run_once(&mut self) {
        let len = self.processes.len();
        let start_instant= Instant::now();
        let mut end_instant= start_instant;
        //Compute the ending instant of the processes pipeline
        for i in 0..len {
            let (proc, instant)= &self.processes[i];
            if let Some(next_instant)= instant.checked_add(proc.get_period()) {
                end_instant= Instant::max(end_instant, next_instant);
            }
        }
        let mut run_finished= false;
        //loop until all processes have been executed
        while run_finished == false {
            let mut next_instant= end_instant;
            // the pipe of this pass; reset at every pass (see NOTE (a) above)
            let mut input_state= None;
            //check what processes needs to be executed
            for i in 0..len {
                //IF the ending instant of the processes pipeline has been reached
                let now= Instant::now();
                let elapsed_time= end_instant.elapsed();
                if elapsed_time > Duration::from_millis(0) {
                    let elapsed_ns= elapsed_time.as_nanos();
                    run_finished= true;
                    break;
                }
                //ELSE it remain processes to execute
                else {
                    let (proc, instant)= &mut self.processes[i];
                    //IF it's time to execute the process
                    if instant.elapsed() > Duration::from_millis(0) {
                        // the returned value becomes the input of the next process executed in this pass
                        // NOTE: `dt` receives the nominal period, not the measured time since the previous execution
                        input_state= proc.exec(&input_state, proc.get_period());
                        // debug trace, printed at every execution of every process
                        println!("{:?} Task executed", proc.get_name());
                        if let Some(nxt_instant)= instant.checked_add(proc.get_period()) {
                            //IF process task take too much time
                            //put next instant as now
                            if nxt_instant.elapsed() > Duration::from_millis(0) {
                                let now= Instant::now();
                                *instant= now;
                                next_instant= now;
                            } else {
                                //update the next instant to execute the process
                                *instant= nxt_instant;
                                //get the closest next instant for process execution
                                next_instant= Instant::min(nxt_instant, next_instant);
                            }
                        }
                    }
                    //ELSE it's not time for this process, so we will check for the next
                    else {
                        //but we check if it's next execution instant is the closest
                        next_instant= Instant::min(*instant, next_instant);
                        continue;
                    }
                }
            }
            if run_finished == false {
                //wait for the remaining time until the very next process execution instant
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
    // 1, and `start_all_side_pipelines` spawns a thread for each of them, which spins forever on an
    // empty pipeline (see `ProcessesPipeline::run_once`, NOTE (b)).
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
    // NOTE: an empty side pipeline never returns from `run_once` (see NOTE (b) there), so its join
    // blocks forever.
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
