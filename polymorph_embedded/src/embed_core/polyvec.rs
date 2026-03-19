use core::{fmt::Error, num::IntErrorKind};

use embassy_stm32::timer::low_level::OutOfRangeError;

use crate::embed_core::utils::VecError;

#[derive(Copy, Clone)]
pub struct PolyVec<T, const NELE: usize> {
    elements: [Option<T>; NELE],
    size: usize
}

impl<T, const NELE: usize> PolyVec<T, NELE> {
    pub fn new_empty() -> Self {
        return Self { elements: core::array::from_fn(|_| None), size: 0 }
    }

    pub fn from_array<I: IntoIterator<Item = T>>(elements: I) -> Self {
        //Fill the polyvec array with default value -> None
        let mut elems = core::array::from_fn(|_| None);
        let mut sz = 0;
        // elements.into_iter() takes ownership of the array and give the ability to iterate over it
        for ele in elements.into_iter() {
            // If the input has more ele than our capacity, 
            if sz >= NELE {
                return Self { 
                    elements: core::array::from_fn(|_| None), 
                    size: 0 
                };
            }
            // Move the ele in the polyvec array of elements
            elems[sz] = Some(ele);
            sz += 1;
        }
        return Self { elements: elems, size: sz };
    }

    pub fn push_back(&mut self, data: T) -> Result<usize, VecError> {
        if self.size + 1 > NELE {
            return Err(VecError::CapacityExceeded)
        } else {
            self.elements[self.size]= Some(data);
            self.size += 1;
            return Ok(self.size)
        }
    }

    pub fn push_range<const nELE: usize>(&mut self, data: [T; nELE]) -> Result<usize, VecError> {
        if self.size + nELE > NELE {
            return Err(VecError::CapacityExceeded)
        } else {
            let mut data= data.map(Some);
            for i in 0..data.len() {
                //.take() put T::default() -> None to the ieme elements
                //  and return the replaced value
                self.elements[self.size]= data[i].take();
                self.size += 1;
            }
            return Ok(self.size);
        }
    }

    pub fn remove(&mut self, index: usize) -> Result<usize, VecError> {
        if self.size as i32 - 1 < 0 || index >= self.size {
            return Err(VecError::OutOfBounds)
        } else {
            for i in index+1..self.size {
                //.take() put T::default() -> None to the ieme elements
                //  and return the replaced value
                self.elements[i-1]= self.elements[i].take();
            }
            self.size -= 1;
            return Ok(self.size)
        }
        
    }

    pub fn remove_range(&mut self, start_index: usize, stop_index: usize) -> Result<usize, VecError> {
        if start_index >= self.size || stop_index > self.size {
            return Err(VecError::OutOfBounds)
        } else {
            for i in start_index..stop_index {
                self.elements[i]= None;
            }
            for i in stop_index..self.size {
                self.elements[start_index+(stop_index-i)]= core::mem::take(&mut self.elements[i]);
            }
            self.size -= stop_index-start_index;
            return Ok(self.size)
        }
    }

    pub fn clear(&mut self) {
        self.elements= core::array::from_fn(|_| None);
        self.size= 0;
    }

    pub fn pop(&mut self, index: usize) -> Result<Option<T>, VecError> {
        if index >= self.size {
            return Err(VecError::OutOfBounds)
        }
        let data= self.elements[index].take();
        self.elements[self.size]= None;
        for i in index+1..self.size {
            //calling take to 
            self.elements[i-1]= self.elements[i].take();
        }
        self.size -= 1;
        return Ok(data)
    }

    pub fn get(&self, index: usize) -> &Option<T> {
        return &self.elements[index];
    }

    pub fn get_mut(&mut self, index: usize) -> &mut Option<T> {
        return &mut self.elements[index];
    }

    pub fn to_slice(&self) -> &[Option<T>] {
        return &self.elements[..self.size];
    }

    pub fn to_mut_slice(&mut self) -> &mut [Option<T>] {
        return &mut self.elements[..self.size];
    }


    pub const fn len(&self) -> usize {
        return self.size;
    }

    pub const fn capacity(&self) -> usize {
        return NELE;
    }
}