# MOLL-E

## Setup
```bash
cd run
```
```bash
docker build -t molle-image .
```

## Motor Control
> On Jetson
```bash
ros2 run moll_e_motor motor_controller_node
```
> On Laptop
```bash
ros2 topic pub /cmd_vel_filtered geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"
```