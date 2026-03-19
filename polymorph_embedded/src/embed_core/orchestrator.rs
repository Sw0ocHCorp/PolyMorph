use core::{cell::RefCell};

use embassy_time::Instant;
use heapless::{FnvIndexMap};

use crate::embed_core::{event_data::EventData, execution::{EmbeddedModule, EmbeddedWorker}, polyvec::PolyVec};

pub struct Orchestrator<'a, const N_WORKERS: usize, const MAX_OBS: usize> {
    workers: FnvIndexMap<u64, EmbeddedWorker<'a, MAX_OBS>, N_WORKERS>,
    execution_graph: PolyVec<u64, N_WORKERS>,
    callbacks_queue: PolyVec<(EventData, PolyVec<u64, MAX_OBS>), N_WORKERS>,
}

impl<'a, const N_WORKERS: usize, const MAX_OBS: usize> Orchestrator<'a, N_WORKERS, MAX_OBS> {
    pub fn new_empty() -> Self {
        Self { workers: FnvIndexMap::new(), execution_graph: PolyVec::<u64, N_WORKERS>::new_empty(),
                callbacks_queue: PolyVec::<(EventData, PolyVec<u64, MAX_OBS>), N_WORKERS>::new_empty() }
    }

    pub fn new() -> Self {
        Self { workers: FnvIndexMap::new(), execution_graph: PolyVec::<u64, N_WORKERS>::new_empty(),
                callbacks_queue: PolyVec::<(EventData, PolyVec<u64, MAX_OBS>), N_WORKERS>::new_empty() }
    }

    pub fn register(&mut self, id: u64, module: &'a mut dyn EmbeddedModule<MAX_OBS>, freq: u64) {
        self.workers.insert(id, EmbeddedWorker::<MAX_OBS>::new(module, freq)).ok();
    }

    pub fn launch_execution_graph(&mut self) -> Option<Instant> {
        let mut next_exec_graph_instant= None;
        for i in 0..self.execution_graph.len() {
            if let Some(j)= self.execution_graph.get(i) && let Some(worker) = self.workers.get_mut(j) {
                //Run the module task of the given worker
                let (inst, result_data)= worker.try_run();
                //Store the next exec instant of the first worker to know when the graph should be executed, the next time
                if next_exec_graph_instant.is_none() {
                    next_exec_graph_instant= Some(inst);
                }
                //If the module task produce callback data that should be passed to some modules to execute their callback function
                if let Some(callback_data) = result_data {
                    self.callbacks_queue.push_back(callback_data);
                }
                //Callback Processing Phase
                if self.callbacks_queue.len() > 0 {
                    //consume all the callbacks data produces by the links between all the modules of the execution graph 
                    while self.callbacks_queue.len() > 0 {
                        let callback_data= self.callbacks_queue.pop(0);
                        if let Ok(callback)= callback_data && let Some(clbk_data)= callback {
                            //Pass the event data to all the modules that should receive this data
                            for m_id in clbk_data.1.to_slice() {
                                if let Some(m_id) = m_id && let Some(worker_callback) =  self.workers.get_mut(m_id) {
                                    if let Some(clbk_data)= worker_callback.launch_module_callback(&clbk_data.0) {
                                        self.callbacks_queue.push_back(clbk_data);
                                    }
                                }
                            }
                        }
                    }
                }
                
            }
        }
        if next_exec_graph_instant.is_none() {
            next_exec_graph_instant= Some(Instant::now());
        }
        return next_exec_graph_instant;
    }

    pub fn set_execution_graph(&mut self, exec_graph: PolyVec<u64, N_WORKERS>) {
        self.execution_graph= exec_graph;
    }
}