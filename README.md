# handsfree_ros_imu

> IMU publisher & visualization demos for **ROS1** and **ROS2**.
> This repo is an English, cleaned-up fork of HANDS-FREE’s original project, adapted so IMU data can be consumed by ROS2 as well.

---

## Overview

This package publishes IMU measurements (linear acceleration, angular velocity and optional orientation) to standard ROS topics so they can be used by mapping, localization and state-estimation nodes (e.g., `robot_localization`, `Cartographer`, `Nav2`).

* **ROS1**: publishes `sensor_msgs/Imu`
* **ROS2**: publishes `sensor_msgs/msg/Imu`
* Includes example launch files, RViz configs and a small demo for replay/testing

> Upstream reference: [https://gitee.com/HANDS-FREE/handsfree_ros_imu.git](https://gitee.com/HANDS-FREE/handsfree_ros_imu.git)

---

## Features

* Serial **or** UDP input (configurable)
* Adjustable publication rate
* Configurable frame IDs and covariances
* Optional yaw-bias removal and simple low‑pass filtering
* Example RViz configuration for quick visualization
* Minimal dependencies

---

## Directory layout

```
.
├── demo/                # example bags & sample configs
├── launch/              # ROS1 and ROS2 launch files
├── rviz/                # RViz configurations
├── scripts/             # helper scripts & parsers
├── tutorials/           # quick HOWTOs
├── CMakeLists.txt
└── package.xml
```

> Names may vary slightly depending on the upstream revision; the layout above reflects what you’ll find in this fork.

---

## Supported platforms

* **ROS1**: Melodic / Noetic (Ubuntu 18.04 / 20.04)
* **ROS2**: Foxy / Humble (Ubuntu 20.04 / 22.04)

The code is simple and should compile on newer distros with zero or minor changes.

---

## Installation

### ROS1 (Melodic / Noetic)

```bash
# 1) Create a catkin workspace
mkdir -p ~/ws_imu/src && cd ~/ws_imu/src

# 2) Clone this repository
git clone https://github.com/marcusvinicius178/handsfree_ros_imu.git

# 3) Resolve dependencies and build
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make

# 4) Source the workspace
source devel/setup.bash
```

### ROS2 (Foxy / Humble)

```bash
# 1) Create a colcon workspace
mkdir -p ~/ws_imu/src && cd ~/ws_imu/src

# 2) Clone this repository
git clone https://github.com/marcusvinicius178/handsfree_ros_imu.git

# 3) Resolve dependencies and build
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

# 4) Source the workspace
source install/setup.bash
```

---

## Parameters (common)

| Param              | Type   | Default        | Description                                         |
| ------------------ | ------ | -------------- | --------------------------------------------------- |
| `port`             | string | `/dev/ttyUSB0` | Serial device (or `udp://host:port`)                |
| `baudrate`         | int    | `115200`       | Serial baudrate                                     |
| `frame_id`         | string | `imu_link`     | TF frame id for the IMU                             |
| `publish_rate`     | double | `100.0`        | Output rate [Hz]                                    |
| `use_orientation`  | bool   | `false`        | Publish orientation (requires reliable calibration) |
| `remove_yaw_bias`  | bool   | `true`         | Enable simple slow yaw-bias removal                 |
| `accel_covariance` | double | `0.02`         | Diagonal covariance for linear acceleration         |
| `gyro_covariance`  | double | `0.02`         | Diagonal covariance for angular velocity            |

> Adjust to match your param file names if they differ; these are conventional defaults used by many IMU nodes.

---

## Usage

### ROS1 (launch)

```bash
# default parameters
roslaunch handsfree_ros_imu imu.launch

# override a few parameters
roslaunch handsfree_ros_imu imu.launch \
  port:=/dev/ttyUSB1 publish_rate:=200.0 frame_id:=imu_link
```

### ROS2 (launch)

```bash
# default parameters
ros2 launch handsfree_ros_imu imu.launch.py

# override a few parameters
ros2 launch handsfree_ros_imu imu.launch.py \
  port:=/dev/ttyUSB1 publish_rate:=200.0 frame_id:=imu_link
```

### Topics

* `/imu/data_raw` → `sensor_msgs/Imu` (linear acceleration + angular velocity)
* `/imu/data`     → `sensor_msgs/Imu` (with orientation if `use_orientation:=true`)

### RViz quick start

```bash
# ROS1
rviz -d $(rospack find handsfree_ros_imu)/rviz/imu.rviz

# ROS2
rviz2 -d $(ros2 pkg prefix handsfree_ros_imu)/share/handsfree_ros_imu/rviz/imu.rviz
```

---

## Calibration (quick guide)

1. Keep the sensor **still on a flat surface** for ~10–20 s to estimate biases.
2. If your IMU provides **orientation**, align `frame_id` with your robot TF (e.g., `base_link` → `imu_link`).
3. Set `use_orientation:=true` only when the device’s orientation output is known to be reliable.

---

## Troubleshooting

* **No data coming in**

  * Check the device is enumerated: `dmesg | grep -i ttyUSB`
  * Add your user to `dialout`: `sudo usermod -a -G dialout $USER` (then re‑login)
  * Confirm `port` and `baudrate` match the IMU
* **Yaw drifting or jumpy**

  * Enable `remove_yaw_bias:=true`
  * Increase `publish_rate` or use a downstream filter (e.g., `robot_localization` EKF/UKF)
* **TF warnings**

  * Ensure `frame_id` exists in your TF tree and that timestamps are consistent

---

## Contributing

Issues and pull requests are welcome. Please keep patches small and focused. For new features, prefer adding them behind parameters so default behavior remains unchanged.

---

## License

The original upstream license applies. Unless otherwise stated in source files, this fork follows the same open‑source license as the HANDS-FREE project. See headers in `scripts/` and `src/` for details.

---

## Acknowledgments

* Based on HANDS-FREE’s IMU package
* Adapted and documented in English by [@marcusvinicius178](https://github.com/marcusvinicius178) for ROS2 consumption
