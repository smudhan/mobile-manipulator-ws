# Mobile Manipulator Mission Control – ROS 2

A ROS 2 Humble simulation of a **mobile manipulator** integrating a **TurtleBot3 Waffle** with a **Doosan A0912 robotic arm** in the AWS RoboMaker Hospital Gazebo environment.

The project demonstrates autonomous navigation using Nav2, arm manipulation through ROS 2 actions, a C++ mission-control state machine, RViz goal selection, LiDAR/IMU/camera sensing, and a software emergency-stop mechanism.

## 1. Project Overview

Normal mission flow:

```text
IDLE
  ↓
First RViz 2D Goal → Navigate to Pickup
  ↓
Pick Object
  ↓
Wait for Place Goal
  ↓
Second RViz 2D Goal → Navigate to Place
  ↓
Place Object
  ↓
Mission Complete → IDLE
```

### Main features

- TurtleBot3 Waffle mobile base
- Doosan A0912 6-DOF arm
- AWS RoboMaker Hospital Gazebo world
- Static map + AMCL localization
- Nav2 autonomous navigation and obstacle avoidance
- LiDAR, IMU and RGB/depth camera
- `ros2_control` joint trajectory control
- C++ Mission Controller using a finite state machine
- ROS 2 Pick and Place actions
- RViz 2D Goal Pose interface
- Software emergency stop/reset

> Pick and place are simulated/mock manipulation actions using predefined arm poses. MoveIt2 is not required.

## 2. Requirements

Tested with:

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic
- Python 3
- C++17
- `colcon`
- `rosdep`

Install the required packages:

```bash
sudo apt update

sudo apt install -y   ros-humble-desktop   ros-humble-navigation2   ros-humble-nav2-bringup   ros-humble-slam-toolbox   ros-humble-gazebo-ros-pkgs   ros-humble-xacro   ros-humble-robot-state-publisher   ros-humble-joint-state-publisher   ros-humble-joint-state-publisher-gui   ros-humble-ros2-control   ros-humble-ros2-controllers   python3-colcon-common-extensions   python3-rosdep
```

If `rosdep` has not been initialized:

```bash
sudo rosdep init
rosdep update
```

## 3. Clone and Build

```bash
mkdir -p ~/assessment_ws/src
cd ~/assessment_ws/src

git clone https://github.com/smudhan/mobile-manipulator-ws

cd ~/assessment_ws

source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

source ~/assessment_ws/install/setup.bash
```

For every new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/assessment_ws/install/setup.bash
```

## 4. Package Structure

```text
assessment_ws/src/
├── aws-robomaker-hospital-world/
├── mission_control/
├── mobile_manipulator_description/
├── my_doosan_pkg/
└── turtlebot3_sim_bringup/
```

### `mobile_manipulator_description`

Combined robot URDF/Xacro, sensors, Nav2 configuration, final map and main launch files.

### `my_doosan_pkg`

Doosan A0912 description, meshes, controllers and supporting launch files.

### `turtlebot3_sim_bringup`

TurtleBot3 simulation and Gazebo/RViz bringup files.

### `aws-robomaker-hospital-world`

Hospital Gazebo environment and its models.

### `mission_control`

Mission FSM, arm controller and Pick/Place action definitions.

```text
mission_control/
├── action/
│   ├── Pick.action
│   └── Place.action
├── include/mission_control/
│   └── mission_state.hpp
└── src/
    ├── arm_controller.cpp
    ├── armteleop.py
    └── mission_controller.cpp
```

## 5. Launch the Simulation

The main launch starts Gazebo, the hospital world, the combined robot, controllers, Nav2 and RViz2.

```bash
source /opt/ros/humble/setup.bash
source ~/assessment_ws/install/setup.bash

ros2 launch mobile_manipulator_description complete.launch.py
```

Wait for Gazebo, RViz and the Nav2 nodes/controllers to finish starting.

## 6. Start Mission Control

Open a second terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/assessment_ws/install/setup.bash

ros2 run mission_control mission_controller
```

Open a third terminal for the arm controller:

```bash
source /opt/ros/humble/setup.bash
source ~/assessment_ws/install/setup.bash

ros2 run mission_control arm_controller
```

The Mission Controller communicates with:

- Nav2 `navigate_to_pose`
- `/pick`
- `/place`
- `/goal_pose`
- `/emergency`

## 7. Run a Pick-and-Place Mission

### Pickup

In RViz, use **2D Goal Pose** and select the first destination.

The first goal is interpreted as the pickup location. Nav2 navigates the robot there.

### Pick

After successful navigation, the Mission Controller sends a `/pick` action.

The arm:

1. Moves to the predefined pickup pose.
2. Performs the simulated grasp.
3. Returns to the home pose.

### Place

Send another **2D Goal Pose** in RViz.

The second goal becomes the place location. The robot navigates there and the Mission Controller sends `/place`.

The arm:

1. Moves to the predefined place pose.
2. Performs the simulated release.
3. Returns home.

The mission then returns to `IDLE`.

## 8. Emergency Stop

Topic:

```text
/emergency
```

Type:

```text
std_msgs/msg/Bool
```

Activate:

```bash
ros2 topic pub --once /emergency std_msgs/msg/Bool "{data: true}"
```

This publishes zero velocity, cancels active Nav2 navigation, enters `EMERGENCY_STOP`, and ignores new RViz goals.

Reset:

```bash
ros2 topic pub --once /emergency std_msgs/msg/Bool "{data: false}"
```

The controller returns to `IDLE` and clears the current mission.

> This is a software simulation E-stop for the mobile navigation. It is not a physical safety-rated hardware E-stop and does not cancel an active arm action.

## 9. Useful ROS 2 Commands

```bash
ros2 node list
ros2 control list_controllers
ros2 topic list
ros2 action list
```

Check sensors:

```bash
ros2 topic echo /scan
ros2 topic echo /odom
ros2 topic echo /joint_states
```

## 10. Important Files

| File | Purpose |
|---|---|
| `mobile_manipulator.urdf.xacro` | Combined TurtleBot3 + Doosan model |
| `scaled_waffle.urdf.xacro` | TurtleBot3 Waffle model |
| `scaled_a0912.xacro` | Doosan A0912 model |
| `nav2.yaml` | Nav2 and AMCL configuration |
| `map4/map4.yaml` | Final static map |
| `complete.launch.py` | Main Gazebo + Nav2 + RViz launch |
| `mission_controller.cpp` | Mission FSM and navigation/action client |
| `arm_controller.cpp` | Pick/place action server and arm control |
| `Pick.action` | Pick action interface |
| `Place.action` | Place action interface |

## 11. Mission State Machine

```text
IDLE
NAVIGATE_TO_PICKUP
PICK
WAIT_FOR_PLACE_GOAL
NAVIGATE_TO_PLACE
PLACE
MISSION_COMPLETE
RECOVERY
EMERGENCY_STOP
```

Normal execution:

```text
IDLE
 → NAVIGATE_TO_PICKUP
 → PICK
 → WAIT_FOR_PLACE_GOAL
 → NAVIGATE_TO_PLACE
 → PLACE
 → MISSION_COMPLETE
 → IDLE
```

`EMERGENCY_STOP` prevents further mission goals until reset.

## 12. Notes

- SLAM was used during development to create the final map; the mission uses the stored static `map4` map with AMCL.
- Navigation uses Nav2 with NavFn planning and Regulated Pure Pursuit control.
- The Doosan arm uses `joint_trajectory_controller`.
- Pick and Place use ROS 2 actions to keep mission logic and arm control modular.
- The main launch configures the Gazebo hospital model paths required by the world.

## Repository

https://github.com/smudhan/mobile-manipulator-ws/
