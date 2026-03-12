use core::{fmt::Error, num::IntErrorKind};

use embassy_stm32::timer::low_level::OutOfRangeError;

use crate::embed_core::utils::VecError;

pub struct PolyVec<T, const N: usize> {
    elements: [Option<T>; N],
    size: usize
}

impl<T, const N: usize> PolyVec<T, N> {
    pub fn new_empty() -> Self {
        return Self { elements: [const {None}; N], size: 0 }
    }

    pub fn from_array<const M: usize>(elements: [T; M]) -> Self {
        //Convert the [T; M] to [Option<T>; M]
        let mut elements= elements.map(Some);
        let mut elems=  [const {None}; N];
        let mut sz= 0;
        if M > N {
            return Self { elements: elems, size: sz }; 
        } else {
            for i in 0..elements.len() {
                //.take() put T::default() -> None to the ieme elements
                //  and return the replaced value
                elems[i]= elements[i].take();
            }
            sz= M;
            return Self { elements: elems, size: sz };
        }
    }

    pub fn push_back(&mut self, data: T) -> Result<usize, VecError> {
        if self.size + 1 > N {
            return Err(VecError::CapacityExceeded)
        } else {
            self.elements[self.size]= Some(data);
            self.size += 1;
            return Ok(self.size)
        }
    }

    pub fn push_range<const M: usize>(&mut self, data: [T; M]) -> Result<usize, VecError> {
        if self.size + M > N {
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

    pub fn to_slice(&self) -> &[Option<T>] {
        return &self.elements[..self.size];
    }

    pub fn to_mut_slice(&mut self) -> &mut [Option<T>] {
        return &mut self.elements[..self.size];
    }


    pub fn len(&self) -> usize {
        return self.size;
    }

    pub fn capacity(&self) -> usize {
        return N;
    }
}