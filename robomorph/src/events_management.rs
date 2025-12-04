use std::{collections::VecDeque, sync::{Arc, Mutex}};

#[derive(Clone)]
/**
 * Observer to receive data from a linked Event
 * Observer can be used in 2 ways:
 * -> Synchrone Way:
 * Using a callback function trigger by the linked Event. 
 * When event trig, it pass the data in the callback functions of all observers attached the event
 * Callback= 
 *      /!\  Send means the variable can be safely send to other threads
 *           Sync means that te variable can be accessed from other threads
 *           dyn any struct implementing T
 * -> Asynchrone Way:
 * Using a Queue/Buffer to store the data send by the Event to each Observers
 *      /!\  The observers can consume the data in their buffers at its own frequency
 */
pub struct Observer<T: Clone + Send + Sync + 'static> {
    callback : Option<Arc<dyn Fn(T) + Send + Sync + 'static>>,
    buffer: Arc<Mutex<VecDeque<T>>>,
}

#[derive(Clone)]
pub struct Event<T: Clone + Send + Sync + 'static> {
    observers: Vec<Observer<T>>
}

impl<T:Clone + Send + Sync + 'static> Event<T> {
    pub fn new_empty() -> Self {
        return Self { observers: vec![] };
    }
    pub fn new(observers: Vec<Observer<T>>) -> Event<T> {
        return Event { observers };
    }

    pub fn plug_observer(&mut self, observer: Observer<T>) {
        self.observers.push(observer);
    }

    pub fn trig(&self, data: T) {
        for observer in &self.observers {
            if let Some(callback) = observer.callback.clone() {
                callback(data.clone());
            }
        }
    }

    pub fn trig_async(&mut self, data: T) {
        for observer in &self.observers {
            if let Ok(mut buffer) = observer.buffer.clone().try_lock() {
                buffer.push_back(data.clone());
            }
            else {
                println!("ERROR: Adding data to observer buffer failed");
            }
        }
    }
}


impl<T: Clone + Send + Sync + 'static> Observer<T> {

    pub fn new(callback: Arc<dyn Fn(T) + Send + Sync>) ->Observer<T> {
        return Observer {callback: Some(callback), buffer: Arc::new(Mutex::new(VecDeque::new()))};
    }
    pub fn new_async() -> Self {
        let mut obs = Observer {
            callback: None,
            buffer: Arc::new(Mutex::new(VecDeque::new())),
        };
        let buffer_clone = obs.buffer.clone();
        obs.callback = Some(Arc::new(move |x: T| {
            if let Ok(mut buffer) = buffer_clone.clone().try_lock() {
                buffer.push_back(x.clone());
            }
        }));
        return obs;
    }

    pub fn is_data_in_buffer(&self) -> bool {
        if let Ok(buffer) = self.buffer.clone().try_lock() {
            return buffer.len() > 0;
        }
        return false;
    }

    pub fn get_incoming_data(&mut self) -> Option<T> {
        if let Ok(mut buffer) = self.buffer.clone().try_lock() {
            return buffer.pop_front();
        }
        else {
            return None;
        }
    }
}