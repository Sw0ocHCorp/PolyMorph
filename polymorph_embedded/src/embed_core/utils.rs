#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum VecError {
    CapacityExceeded,
    OutOfBounds,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PolyError {
    ErrorAddingObserver,
    ErrorSensorDisconnected,
    ErrorSensorMeasurements,
    ErrorSendingMessage,
    ErrorReceivingMessage, 
    ErrorLossConnection
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ModuleStatus {
    Alive,
    WaitingForData,
    Dead,
}