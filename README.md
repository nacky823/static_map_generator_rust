# static_map_generator_rust
この ROS 2 パッケージは、[ogm_flow_estimator_static](https://github.com/ryuichiueda/ogm_flow_estimator_static.git) を理解するために作成した練習用パッケージです。[ogm_flow_estimator_static の実装](https://github.com/ryuichiueda/ogm_flow_estimator_static/tree/master/src)を参考にして作成しました。公開してくださり、ありがとうございます。

## Overview
一定位置に静止した状態で取得したレーザスキャンデータを用い、
静的な障害物のみを反映した環境地図を生成する。

## Tested Environment
+ ROS 2 distribution: Humble
+ Rust version: 1.81.0
    + Uses the [ros2_rust](https://github.com/ros2-rust/ros2_rust.git) library

## Installation
任意のワークスペースで以下を実行
```
git clone https://github.com/nacky823/scan_to_map_rust.git src/scan_to_map_rust
git clone https://github.com/nacky823/static_map_generator_rust.git src/static_map_generator_rust
source /opt/ros/humble/setup.bash
colcon build --packages-up-to scan_to_map_rust static_map_generator_rust
source install/setup.bash
```

## How to Run
`/scan`トピックがパブリッシュされている状態で以下を実行
```
ros2 launch static_map_generator_rust scan_to_static_map.launch.py
```

## Input/Output
| **Node Name**               | **Input** | **Message Type**                  | **Output** | **Message Type**                    |
|----------------------------|------------------------------|-----------------------------------|------------------------------|-------------------------------------|
| `/scan_to_map_node`         | `/scan`                      | `sensor_msgs/msg/LaserScan`       | `/scan_map`                  | `nav_msgs/msg/OccupancyGrid`        |
| `/static_map_generator_node`| `/scan_map`                  | `nav_msgs/msg/OccupancyGrid`      | `/static_map`                | `nav_msgs/msg/OccupancyGrid`        |

## References
+ [https://github.com/ros2-rust/ros2_rust.git](https://github.com/ros2-rust/ros2_rust.git)
+ [https://github.com/ryuichiueda/ogm_flow_estimator_static.git](https://github.com/ryuichiueda/ogm_flow_estimator_static.git)