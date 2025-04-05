# MOLL-E

## Setup
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
ros2 run molle_motor motor_controller_node
```
> On Laptop
```bash
ros2 topic pub /cmd_vel_filtered geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"
```

### Motor Control

- **Left Speed = linear.x + (turn_scale × angular.z)**
- **Right Speed = linear.x − (turn_scale × angular.z)**

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
