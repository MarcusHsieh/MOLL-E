# MOLL-E

## Setup

```
git clone --recursive https://github.com/MarcusHsieh/MOLL-E.git
```
```bash
cd run
```
```bash
docker build -t molle-image .
```
```bash
./run/jetson.sh
```

## Motors
> On Jetson
```bash
ros2 run motor_pkg motor_controller_node
```
> On Laptop
```bash
ros2 topic pub /cmd_vel_filtered geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"
```

### Motor Control

- **Left Speed = linear.x + (turn_scale × angular.z)**
- **Right Speed = linear.x − (turn_scale × angular.z)**

> `linear.x` = magnitude (-) back (+) forward
> `angular.z` = rotation (-) left (+) right

```bash
turn_scale = 1.0
```

| linear.x | angular.z | Left Speed = linear.x + angular.z | Right Speed = linear.x − angular.z | Behavior                                            |
|----------|-----------|-----------------------------------|------------------------------------|-----------------------------------------------------|
| 0.0      | 1.0       | 0.0 + 1.0 = **1.0**               | 0.0 − 1.0 = **-1.0**               | Rotate in place counterclockwise (turn left)        |
| 0.0      | -1.0      | 0.0 + (-1.0) = **-1.0**           | 0.0 − (-1.0) = **1.0**             | Rotate in place clockwise (turn right)              |
| 0.5      | 0.0       | 0.5 + 0.0 = **0.5**               | 0.5 − 0.0 = **0.5**                | Move straight forward                               |
| 0.5      | 0.5       | 0.5 + 0.5 = **1.0**               | 0.5 − 0.5 = **0.0**                | Move forward while turning left (curve left)        |
| 0.5      | -0.5      | 0.5 + (-0.5) = **0.0**            | 0.5 − (-0.5) = **1.0**             | Move forward while turning right (curve right)      |
| -0.5     | 0.0       | -0.5 + 0.0 = **-0.5**             | -0.5 − 0.0 = **-0.5**              | Move straight backward                              |
| -0.5     | 0.5       | -0.5 + 0.5 = **0.0**              | -0.5 − 0.5 = **-1.0**              | Move backward while turning right (curve right)     |
| -0.5     | -0.5      | -0.5 + (-0.5) = **-1.0**          | -0.5 − (-0.5) = **0.0**            | Move backward while turning left (curve left)       |

| Nodes | Topics |
| --- | --- |
| /motor_controller_node | /cmd_vel_filtered |

## LIDAR

Start LIDAR Node
```bash
ros2 launch ldlidar_sl_ros2 ld14p.launch.py 
```

Launch rviz2
```bash
rviz2 -d src/LIDAR_pkg/rviz2/ldlidar.rviz
```
| Nodes | Topics |
| --- | --- |
| /base_link_to_base_laser_ld14p | /pointcloud2d |
| /ldlidar_publisher_ld14 | /scan |
| | /tf_static|

## IMU
```bash
ros2 launch mpu6050driver mpu6050driver_launch.py
```
| Nodes | Topics |
| --- | --- |
| /mpu6050driver_node | /imu |