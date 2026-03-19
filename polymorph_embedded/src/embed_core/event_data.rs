#[derive(PartialEq)]
pub enum ModuleID {
    
}

#[derive(PartialEq)]
pub enum EventData {
    NoMeasurements,
    TrigPin,
    ImuMeasurements,
}

impl EventData {
    pub fn to_json(&mut self) {

    }
}
