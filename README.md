# ufomap_ros2
ROS 2 port of [ufomap](https://github.com/UnknownFreeOccupied/ufomap/)

Recieves sensor_msgs::msg::PointCloud2 and sends {some useful form of voxel occupancy grid (haven't gotten that far lol)}

## Build instructions
1. Navigate to (or create) your ROS 2 workspace:
- `cd /robot_ws`
2. Create and go to src directory:
- `mkdir src && cd src`
3. Clone this repository:
- `git clone https://github.com/connortynan/ufomap_ros2`
4. Build from workspace directory:
- `cd ../ && colcon build`
