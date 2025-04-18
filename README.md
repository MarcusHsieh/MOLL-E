# MOLL-E
![image](https://github.com/user-attachments/assets/3d2b0bf3-9881-4387-bd31-23d83d10e334)

![image](https://github.com/user-attachments/assets/3ef348bf-9c62-419e-a6e3-6ba8ab5e9505)


## Docker
Working: Dockerfile-Num3 @ molle-num3

## Setup

```
git clone --recursive https://github.com/MarcusHsieh/MOLL-E.git
```
```bash
source /opt/ros/humble/install/setup.bash
```
```bash
cd run
```
> -t = image name  
> -f = Dockerfile name  
```bash
docker build -t molle-image -f Dockerfile-Num3 .
```
```bash
./run/jetson.sh
```
> Build
```bash
colcon build
```
> Source
```bash
source install/setup.bash
```
> Launch everything
```bash
ros2 launch molle_launch molle.launch.py
```

## Motors
> Start motor node(Jetson)
```bash
ros2 run motor_ros2 motor_controller_node
```
> Testing (Laptop)
```bash
ros2 topic pub /cmd_vel_filtered geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"
```

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

> Start LIDAR node (Jetson)
```bash
ros2 launch ldlidar_sl_ros2 ld14p.launch.py 
```

> Launch rviz2 (Laptop)
```bash
rviz2 -d src/ldlidar_sl_ros2/rviz2/ldlidar.rviz
```
| Nodes | Topics |
| --- | --- |
| /base_link_to_base_laser_ld14p | /pointcloud2d |
| /ldlidar_publisher_ld14 | /scan |
| | /tf_static|

## IMU
> Start mpu6950 node (Jetson)
```bash
ros2 launch mpu6050driver mpu6050driver_launch.py
```
| Nodes | Topics |
| --- | --- |
| /mpu6050driver_node | /imu |

## Rosboard
> Start websocket @ <ip>:8888
```bash
ros2 run rosboard rosboard_node
```

## Other
> See all nodes graph (Laptop)
```bash
ros2 run rqt_graph rqt_graph
```
> See TF Tree (Laptop)
```bash
ros2 run tf2_tools view_frames
```
