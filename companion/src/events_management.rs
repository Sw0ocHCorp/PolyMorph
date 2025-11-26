use std::sync::Arc;

#[derive(Clone)]
pub struct Observer<T: Clone + Send + Sync> {
    callback : Arc<Box<dyn Fn(T) + Send + Sync>>
}

#[derive(Clone)]
pub struct Event<T: Clone + Send + Sync> {
    observers: Vec<Observer<T>>
}


impl<T:Clone + Send + Sync> Event<T> {
    pub fn new(observers: Vec<Observer<T>>) -> Event<T> {
        return Event { observers };
    }

    pub fn plug_observer(&mut self, observer: Observer<T>) {
        self.observers.push(observer);
    }

    pub fn trig(&self, data: T) {
        for observer in &self.observers {
            (observer.callback)(data.clone());
        }
    }
}

impl<T: Clone + Send + Sync> Observer<T> {
    pub fn new(callback: Arc<Box<dyn Fn(T) + Send + Sync>>) ->Observer<T> {
        return Observer {callback};
    }
}