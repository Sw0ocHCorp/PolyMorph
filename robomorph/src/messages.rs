use ordered_float::OrderedFloat;
use std::collections::HashMap;

#[derive(Clone)]
pub enum Message {
    Sentence(String),
    Frame(Vec<u8>),
    Image(),
    LidarMeasurements(HashMap<OrderedFloat<f64>, f64>)
}