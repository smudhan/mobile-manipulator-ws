# TurtleBot3 Simulation Bringup Package

A ROS 2 package for launching TurtleBot3 in Gazebo simulation with RViz visualization. This package provides launch files for differential drive control using `cmd_vel` topics.

## Package Contents

### Launch Files

- **`bringup.launch.py`**: Main launch file that starts both Gazebo and RViz
- **`gazebo.launch.py`**: Launches TurtleBot3 in Gazebo simulation
- **`rviz.launch.py`**: Launches RViz2 with TurtleBot3 visualization config

### Configuration Files

- **`config/turtlebot3.rviz`**: RViz configuration with robot model, TF, laser scan, and camera displays

## Prerequisites

Ensure you have the following packages installed:

```bash
sudo apt update
sudo apt install ros-<distro>-turtlebot3-gazebo ros-<distro>-turtlebot3-description
```

Replace `<distro>` with your ROS 2 distribution (e.g., `humble`, `foxy`, `iron`).

## Building the Package

1. Navigate to your workspace:
```bash
cd ~/your_workspace
```

2. Build the package:
```bash
colcon build --packages-select turtlebot3_sim_bringup
```

3. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Set TurtleBot3 Model

Before launching, set the TurtleBot3 model (burger, waffle, or waffle_pi):

```bash
export TURTLEBOT3_MODEL=burger
```

Add this to your `~/.bashrc` to make it persistent.

### Launch Options

#### 1. Launch Everything (Gazebo + RViz)

```bash
ros2 launch turtlebot3_sim_bringup bringup.launch.py
```

#### 2. Launch Only Gazebo

```bash
ros2 launch turtlebot3_sim_bringup gazebo.launch.py
```

#### 3. Launch Only RViz

```bash
ros2 launch turtlebot3_sim_bringup rviz.launch.py
```

### Launch Arguments

You can customize the launch with the following arguments:

- `use_sim_time`: Use simulation time (default: `true`)
- `x_pose`: Initial x position of the robot (default: `0.0`)
- `y_pose`: Initial y position of the robot (default: `0.0`)

Example with custom position:

```bash
ros2 launch turtlebot3_sim_bringup bringup.launch.py x_pose:=1.0 y_pose:=2.0
```

## Controlling the Robot

The TurtleBot3 uses differential drive control and listens to velocity commands on the `/cmd_vel` topic.

### Using Keyboard Teleop

Install and run the teleop keyboard package:

```bash
sudo apt install ros-<distro>-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Publishing Commands Manually

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

## Topics

Key topics for robot control and monitoring:

- `/cmd_vel`: Velocity commands (geometry_msgs/Twist)
- `/odom`: Odometry data (nav_msgs/Odometry)
- `/scan`: Laser scan data (sensor_msgs/LaserScan)
- `/camera/image_raw`: Camera image (sensor_msgs/Image)
- `/imu`: IMU data (sensor_msgs/Imu)

## RViz Configuration

The included RViz configuration displays:

- Robot model (URDF visualization)
- TF transforms
- Laser scan data
- Grid reference
- Camera view (optional, disabled by default)

## Notes

- This package does **not** include autonomous navigation capabilities
- For autonomous navigation, use separate packages like `nav2` or custom navigation implementations
- The differential drive controller is built into the TurtleBot3 Gazebo model

## License

Apache-2.0

## Maintainer

User <user@todo.todo>
