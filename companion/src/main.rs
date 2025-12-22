use robomorph::{communication::UDPChannel, lidar::LidarPerceptionManager};

fn main() {
    let udp= Some(UDPChannel::new_async("127.0.0.1", 8090, "127.0.0.1", 8080));
    let lidar_perception_management= LidarPerceptionManager{};
    println!("Hello, world!");
}
