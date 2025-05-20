
# 🧭 odom_computation (ROS 2 Humble)

![GitHub repo size](https://img.shields.io/github/repo-size/vigneshnaidu03/odom_computation)
![GitHub stars](https://img.shields.io/github/stars/vigneshnaidu03/odom_computation?style=social)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)
![ROS2-Humble](https://img.shields.io/badge/ROS2-Humble-blue)

> Compute odometry from simulated wheel encoder ticks using differential drive kinematics — made for embedded robotics simulation, tested with ROS 2 Humble.

---

## 📦 Package Overview

This ROS 2 Python package simulates a differential drive robot and computes its odometry using encoder tick values. It includes:

- ✅ Simulated wheel tick publisher  
- ✅ Odometry calculator using differential drive logic  
- 🔄 Publishes to `/odom` in `nav_msgs/msg/Odometry` format  
- ⚙ Robot parameters: wheel radius, base width, and ticks per revolution  

---

## 🗂 Directory Structure

```
odom_computation/
├── odom_computation/
│   ├── __init__.py
│   ├── wheel_tick_pub_ros2.py     # Simulates encoder ticks
│   └── odom_calc_node.py          # Converts ticks to Odometry
├── launch/
│   └── odom_system.launch.py      # Launches both nodes
├── setup.py
├── package.xml
└── README.md
```

---

## 🧠 Robot Parameters

| Parameter               | Value                                      |
|------------------------|--------------------------------------------|
| `wheel_radius`         | 0.065 m                                    |
| `wheel_base`           | 0.21 m                                     |
| `ticks_per_revolution` | 512                                        |
| Tick topics            | `/left_wheel_ticks`, `/right_wheel_ticks` |
| Odometry topic         | `/odom`                                    |

---

## 🚀 Installation

### 1. Clone the repo into your ROS 2 workspace

```bash
cd ~/ros2_ws/src
git clone https://github.com/vigneshnaidu03/odom_computation.git
```

### 2. Build and source

```bash
cd ~/ros2_ws
colcon build --packages-select odom_computation
source install/setup.bash
```

---

## ▶ Run the Package

```bash
ros2 launch odom_computation odom_system.launch.py
```

This will launch two nodes:

- `wheel_tick_pub_ros2.py` → Simulates wheel encoder ticks  
- `odom_calc_node.py` → Computes and publishes odometry to `/odom`

---

## 🔍 Check Output

You can monitor the odometry data with:

```bash
ros2 topic echo /odom
```

Expected behavior:

- `x` increases over time (robot moving forward)  
- `y ≈ 0`  
- `θ ≈ 0` (no rotation)  
- Orientation in quaternion  
- Twist field shows linear & angular velocity  

---

## 🧪 Optional Extensions

Ideas to expand this package:

- Add an `imu_publisher.py` node to publish synthetic IMU data  
- Visualize in **RViz2**  
- Record rosbag logs for analysis  

---

## 📚 References

- [ROS 2 Python Publisher & Subscriber Tutorial](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)  
- [nav_msgs/msg/Odometry](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html)  
- [sensor_msgs/msg/Imu](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html)  
- [tf_transformations for quaternions](https://github.com/ros/geometry2)  

---

## 👨‍💻 Author

**Vignesh** 
Embedded Robotics Enthusiast 
[GitHub](https://github.com/vigneshnaidu03)

---

## 📝 License

This project is open source under the [MIT License](LICENSE).
