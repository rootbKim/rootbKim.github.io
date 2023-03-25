---
layout: post
title: UR GAZEBO Simulation 설치하기
feature-img: "assets/img/portfolio/openai.png"
thumbnail: "assets/img/portfolio/openai.png"
tags: [Gym Simulation]
---

Gym을 이용한 시뮬레이션 환경을 구성하기 위하여 UR 로봇에서 제공하는 ROS2 Gazebo Simulation 패키지를 설치하고, CLI를 이용하여 로봇을 컨트롤한다.

## 1. UR GAZEBO Simulation 환경 설치

ROS2 Galactic 버전에서 사용이 가능한 것으로 나와있어, Galactic 버전을 사용하여 설치한다.

```bash
mkdir -p ur_ws/src
cd ur_ws/src

git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git -b ros2
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git -b galactic
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git -b galactic
git clone https://github.com/UniversalRobots/Universal_Robots_Client_Library.git
git clone https://github.com/ros-controls/ros2_control.git -b galactic
git clone https://github.com/ros-controls/ros2_controllers.git -b galactic
sudo apt-get install ros-galactic-ur-msgs
```

rosdep 및 빌드

```bash
source /opt/ros/galactic/setup.bash
rosdep install --ignore-src --from-paths src -y -r       # install also is there are unreleased packages
colcon build --symlink-install
```

시뮬레이션 실행

```bash
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py
```

joint controller 명령

```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 0, nanosec: 0}}]}" --once
```

TEST launch 실행

```bash
# 초기 위치 셋팅
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], points: [{positions: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0], time_from_start: {sec: 4, nanosec: 0}}]}" --once

# test launch 실행
ros2 launch ur_bringup test_joint_trajectory_controller.launch.py
```

## 참고문헌

- [UR ROS2 Gazebo Simulation github pages](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation)
- [ros2_controller joint_trajectory_controller](https://control.ros.org/foxy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html#using-joint-trajectory-controller-s)