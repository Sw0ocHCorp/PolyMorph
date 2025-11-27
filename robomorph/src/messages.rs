#[derive(Clone)]
pub enum Message {
    Sentence(String),
    Frame(Vec<u8>),
    Image(),
    LidarMeasurements(Vec<f32>)
}