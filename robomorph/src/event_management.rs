use std::{collections::VecDeque, sync::{Arc, Mutex}};


#[derive(Clone)]
pub struct Observer<T: Clone + Send + Sync + 'static> {
    is_async: bool,
    callback : Option<Arc<Mutex<dyn FnMut(T) + Send + Sync + 'static>>>,
    buffer: Arc<Mutex<VecDeque<T>>>,

}

impl<T:Clone + Send + Sync + 'static> Observer<T> {
    pub fn new(callback: Arc<Mutex<dyn FnMut(T) + Send + Sync>>) ->Observer<T> {
        return Observer {is_async: false, callback: Some(callback), buffer: Arc::new(Mutex::new(VecDeque::new()))};
    }

    pub fn new_async() -> Self {
        let mut obs = Observer {
            is_async: true,
            callback: None,
            buffer: Arc::new(Mutex::new(VecDeque::new()))
        };
        let buffer_clone = obs.buffer.clone();
        obs.callback = Some(Arc::new(Mutex::new(move |x: T| {
            if let Ok(mut buffer) = buffer_clone.clone().try_lock() {
                buffer.push_back(x.clone());
            }
        })));
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

    pub fn put_data_in_buffer(&self, data: T) {
        if let Ok(mut buffer) = self.buffer.clone().try_lock() {
            buffer.push_back(data.clone());
        }
    }
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
            if observer.is_async {
                if let Ok(mut buffer) = observer.buffer.clone().try_lock() {
                    buffer.push_back(data.clone());
                }
            } else {
                if let Some(mut clbk) = observer.callback.clone() {
                    if let Ok(mut callback)= clbk.clone().try_lock() {
                        callback(data.clone());
                    }
                }
            }
        }
    }
}