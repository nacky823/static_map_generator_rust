use rclrs;
use nav_msgs::msg::OccupancyGrid;
use std::sync::{Arc, Mutex};
use std::time::Duration;

fn generate_static_map(buffer: &Vec<OccupancyGrid>) -> Option<OccupancyGrid> {
    if buffer.len() < 3 {
        return None;
    }

    let mut result = buffer.last()?.clone();
    let size = result.data.len();

    // 各セルの出現回数カウント用
    let mut counter = vec![0u32; size];

    for map in buffer.iter() {
        for (i, v) in map.data.iter().enumerate() {
            if *v > 0 {
                counter[i] += 1;
            }
        }
    }

    let threshold = (buffer.len() as u32 * 7) / 10;
    result.data.iter_mut().enumerate().for_each(
        |(i, v)| {
            *v = if counter[i] >= threshold { 100 } else { 0 };
        }
    );

    Some(result)
}

fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(std::env::args())?;
    let node = rclrs::Node::new(&context, "static_map_generator")?;

    let map_buffer = Arc::new(Mutex::new(Vec::<OccupancyGrid>::new()));
    let map_buffer_sub = Arc::clone(&map_buffer);

    let _sub = node.create_subscription("scan_map", rclrs::QOS_PROFILE_DEFAULT, 
        move |msg: OccupancyGrid| {
            let mut buf = map_buffer_sub.lock().unwrap();
            buf.push(msg);
            if buf.len() > 30 {
                buf.remove(0); // 最大30個保持
            }
        }
    )?;

    let publisher = node.create_publisher::<OccupancyGrid>("static_map", rclrs::QOS_PROFILE_DEFAULT)?;
    let publisher = Arc::new(publisher);
    let publisher_loop = Arc::clone(&publisher);
    let map_buffer_loop = Arc::clone(&map_buffer);

    std::thread::spawn(move || {
        loop {
            std::thread::sleep(Duration::from_secs(1));
            let buffer = map_buffer_loop.lock().unwrap();
            if let Some(static_map) = generate_static_map(&buffer) {
                publisher_loop.publish(static_map).unwrap();
            }
        }
    });

    println!("Generating /static_map from /scan_map buffer...");
    rclrs::spin(node)
}
