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

    let threshold = (buffer.len() as u32 * 70) / 100;
    result.data.iter_mut().enumerate().for_each(|(i, v)| {
        *v = if counter[i] >= threshold { 100 } else { 0 };
    });

    Some(result)
}
