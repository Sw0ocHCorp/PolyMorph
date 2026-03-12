#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum VecError {
    CapacityExceeded,
    OutOfBounds,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PolyError {
    ErrorAddingObserver,
}