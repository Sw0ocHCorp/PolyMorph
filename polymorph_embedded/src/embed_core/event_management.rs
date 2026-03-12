use core::cell::RefCell;

use embassy_stm32::timer::low_level::OutOfRangeError;

use crate::embed_core::{polyvec::PolyVec, utils::PolyError};


pub trait EmbeddedObserver<T> {
    //The main task of the Observer (callback function)
    fn exec_answer(&mut self, data: &T);
    //Knowing if the are problem on the observer
    //  If true -> means that the observer can run it's task
    //  If false -> means that it not run because of deconnection or whatever
    //      it require to check for incomming reconnection or let the Observer in a dead mode
    fn is_alive(&self) -> bool;
    fn set_alive(&mut self, status: bool);
}

//'a statement force the compiler to consider that all the Observers will live as longer as the Publisher
//  Remove an Observer in the list will cause panic or compilator error
//  Because every Observers put in the list need to live as longer as the Publisher 
///!\ All the struct that will contain a Publisher will need to have the 'a statement
//      To say that the struct containing a publisher will live as longer as the publisher 
pub struct EmbeddedEventPublisher<'a, T, const N: usize> {
    observers: PolyVec<&'a mut dyn EmbeddedObserver<T>, N>
}

impl<'a, T, const N: usize> EmbeddedEventPublisher<'a, T, N> {
    pub fn new_empty() -> Self {
        return Self { observers: PolyVec::<&'a mut dyn EmbeddedObserver<T>, N>::new_empty() };
    }
    
    pub fn new<const M: usize>(obs: [&'a mut dyn EmbeddedObserver<T>; M]) -> Self {
        return Self { observers: PolyVec::<&'a mut dyn EmbeddedObserver<T>, N>::from_array(obs) };
    }

    pub fn publish(&mut self, data: &T) {
        for obs in self.observers.to_mut_slice() {
            if let Some(o) = obs {
                o.exec_answer(&data);
            }
        }
    }

    pub fn attach_observer(&mut self, observer: &'a mut dyn EmbeddedObserver<T>) -> Result<usize, PolyError> {
        match self.observers.push_back(observer) {
            Ok(n_observers) => return Ok(n_observers),
            Err(_) => return Err(PolyError::ErrorAddingObserver),
        }
        
    }
}