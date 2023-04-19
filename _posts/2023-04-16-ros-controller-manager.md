---
layout: post
title: ROS Controller Manager 이해하기
feature-img: "assets/img/posts/ros.png"
thumbnail: "assets/img/posts/ros.png"
tags: [ROS2 GAZEBO]
---

Gazebo에 spawn된 로봇 또는 ROS2 기반의 실제 로봇의 관절을 제어하기 위한 `controller_manager`에 대해서 알아본다.

## 1. Controller Manager란?

ROS2의 Controller Manager는 하드웨어 인터페이스와 로봇 제어 알고리즘 사이의 인터페이스를 제공하는 ROS2 패키지로서, 제어기 프로그램을 로드하고 실행하는 방법을 제공하며, 제어기 프로그램은 로봇의 다양한 하드웨어를 제어할 수 있다. 이러한 하드웨어는 관절, 휠, 손잡이, 포스 센서, 비전 센서 등 다양한 유형의 로봇 부품일 수 있다.

Controller Manager는 ROS2의 pluginlib 라이브러리를 사용하여 제어기를 로드하고 실행합니다. 이를 통해 사용자는 컴파일 시간에 모든 제어기를 빌드할 필요 없이 런타임에 필요한 제어기만 로드할 수 있다.

## 2. Architecture

다음은 ros2_control framework의 아키텍처이다.

<img src="/assets/img/posts/230416_ros_control_architecture.png">

### 2.1 Controller Manager

Controller Manager(CM)는 ros2_control 프레임워크의 컨트롤러와 하드웨어 추상화 부분을 연결한다. 또한 ROS 서비스를 통해 사용자의 entry-point로 작동한다. CM은 사용자 정의 설정에 통합될 수 있도록 `executor`가 없는 노드로 구현된다. (일반 사용자는 controller_manager 패키지의 ros2_control_node 파일에 구현된 기본 노드 설정을 사용하는 것이 좋다.)

CM은 컨트롤러와 컨트롤러가 필요로하는 인터페이스를 관리한다(로딩, 활성화, 비활성화, 언로딩). 그리고 하드웨어 구성 요소(인터페이스)에 접근할 수 있다. 컨트롤러 매니저는 필요한 인터페이스와 제공되는 인터페이스를 매치하고, 활성화될 때 컨트롤러가 하드웨어에 접근할 수 있도록 하거나 액세스 충돌이 있는 경우 오류를 보고한다.

컨트롤 루프의 실행은 CM의 `update()` 메소드에 의해 관리됩니다. 이 메소드는 하드웨어 구성 요소로부터 데이터를 읽고, 모든 활성 컨트롤러의 출력을 업데이트하고, 결과를 구성 요소에 기록한다.

### 2.2 Resource Manager

Resource Manager(RM)는 ros2_control 프레임워크에서 물리적 하드웨어와 하드웨어 드라이버(하드웨어 구성 요소)를 추상화한다. RM은 `pluginlib` 라이브러리를 사용하여 구성 요소를 로드하고, 그들의 라이프사이클과 구성 요소의 상태 및 명령 인터페이스를 관리한다. RM에 의한 이러한 추상화는 구현된 하드웨어 구성 요소(로봇 및 그립퍼 등)의 재사용을 가능하게 하며, 상태 및 명령 인터페이스에 대한 유연한 하드웨어 응용 프로그램(모터 제어 및 엔코더 리딩에 대한 별도의 하드웨어/통신 라이브러리 등)을 가능하게 한다.

제어 루프 실행에서는 RM의 `read()` 및 `write()` 메서드가 하드웨어 구성 요소와의 통신을 다룬다.

### 2.3 Controller

ros2_control 프레임워크의 컨트롤러는 제어 이론에서 정의된 기능을 가진다. 컨트롤러는 기준 값과 측정 출력을 비교하고 이 오차를 기반으로 시스템의 입력을 계산한다. 컨트롤러는 ControllerInterface에서 pluginlib 라이브러리를 사용하여 파생된 객체이며 플러그인으로 내보내진다.

제어 루프를 실행할 때는 `update()` 메서드가 호출된다. 이 메서드는 최신 하드웨어 상태에 액세스하고 컨트롤러가 하드웨어의 명령 인터페이스를 쓸 수 있도록 한다.


### 2.4 User Interfaces

사용자는 컨트롤러 매니저의 `service`를 사용하여 ros2_control 프레임워크와 상호 작용한다. 서비스 호출은 CLI 또는 ROS 노드를 통해 직접 사용할 수 있지만, ros2 cli와 통합된 사용자 친화적인 Command Line Interface(CLI)가 존재한다. 이는 자동 완성을 지원하며 일반적인 명령어가 제공됩니다. 기본 명령은 ros2 control이며, CLI 기능에 대한 설명은 [ros2controlcli](https://github.com/ros-controls/ros2_control/tree/master/ros2controlcli) 패키지를 참조하면 된다.

### 2.5 Hardware Components

하드웨어 컴포넌트는 물리적 하드웨어와의 통신을 구현하고 그 추상화를 ros2_control 프레임워크에서 표현한다. 컴포넌트는 pluginlib 라이브러리를 사용하여 플러그인으로 내보내야 하고, 리소스 매니저는 이러한 플러그인을 동적으로 로드하고 lifecycle을 관리한다.

하드웨어 컴포넌트에는 다음과 같은 세 가지 기본 유형이 있다.

#### System

다중 DOF(자유도)를 갖는 산업용 로봇과 같은 복잡한 로봇 하드웨어를 뜻하며, 액추에이터 구성요소와의 주요 차이점은 인간형 로봇의 손과 같은 복잡한 변속장치를 사용할 수 있다는 점이다. 이 컴포넌트는 읽기 및 쓰기 기능을 갖고 있다. 하드웨어와의 단일 논리적 통신 채널이 있는 경우(예: KUKA-RSI)에 사용됩니다.

#### Sensor

주변 환경을 감지하는 데 사용됩니다. 센서 컴포넌트는 조인트(예: 인코더) 또는 링크(예: 힘-토크 센서)와 관련이 있으며, 읽기만 가능합니다.

#### Actuator

모터, 밸브 및 유사한 단순한(1 DOF) 로봇 하드웨어를 뜻하며, 액추에이터 구현은 하나의 조인트에만 관련이 있다. 이 컴포넌트 유형은 읽기 및 쓰기 기능을 갖고 있다. 가능하지 않은 경우 읽기는 필수가 아닐 수 있다(예: 아두이노 보드로 DC 모터 제어). 액추에이터 유형은 각 모터가 독립적으로 CAN 통신을 지원하는 경우 멀티 DOF 로봇에서도 사용할 수 있다.

하드웨어 컴포넌트의 자세한 설명은 [Hardware Access through Controllers](https://github.com/ros-controls/roadmap/blob/master/design_drafts/hardware_access.md) 설계 문서에서 제공됩니다.

## 3. ros2_controllers

ros2_control framework에서 사용할 수 있도록 만들어진 라이브러리로 다양한 controller를 제공한다.

ros2_control 프레임워크는 컨트롤러를 커맨드 인터페이스 유형에 따라 분류하기 위해 다음과 같은 네임스페이스를 사용한다. 컨트롤러는 공통 하드웨어 인터페이스 정의를 사용하며, 컨트롤러의 네임스페이스는 다음과 같은 커맨드 인터페이스 유형을 명령한다

* `position_controllers`: `hardware_interface::HW_IF_POSITION`
* `velocity_controller`: `hardware_interface::HW_IF_VELOCITY`
* `effort_controllers`: `hardware_interface::HW_IF_ACCELERATION`
* `effort_controllers`: `hardware_interface::HW_IF_EFFORT`

### 3.1 사용 가능한 controller 리스트

* `Admittance Controller`
* `Tricycle Controller`
* `Differential Drive Controller`
* `Forward Command Controller`
* `Joint Trajectory Controller`
* `Position Controllers`
* `Velocity Controllers`
* `Effort Controllers`

### 3.2 사용 가능한 broadcaster 리스트

* `Joint State Broadcaster`
* `Imu Sensor Broadcaster`
* `Force Torque Sensor Broadcaster`

## 4. ros2_control 예제

사용 예에 대한 페이지는 다음을 참고한다.

* [ros2_control Demo](https://control.ros.org/master/doc/ros2_control_demos/doc/index.html#examples)
* [ros-controls/ros2_control_demos](https://github.com/ros-controls/ros2_control_demos)
* [ros-controls/roscon2022_workshop](https://github.com/ros-controls/roscon2022_workshop)

## 5. UR ros2_control 예제

지난 포스트에서 설치한 ur 환경을 이용한 ros2_control 사용 예를 정리한다.

먼저 시뮬레이션을 실행한다.

```bash
cd ur_ws
source install/setup.bash

ros2 launch ur_simulation_gazebo ur_sim_control.launch.py
```

시뮬레이션 launch 파일에 `controller_manager` 노드가 포함되어 있고, arguments로 `initial_joint_controller`를 사용한다.

```python
    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(start_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(start_joint_controller),
    )
```

이 `initial_joint_controller`는 `joint_trajectory_controller`가 디폴트 controller로 지정되어 있으므로, `joint_trajectory_controller`가 실행된다.

```python
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
```

controller 설정 파일인 `ur_controllers.yaml` 파일을 보면, 다음과 같이 사용할 controller에 대한 설정이 되어있고, 이 중에서 `joint_trajectory_controller`를 가져다가 사용하는 것이다. 이외에도 controller를 바꾸면 다음에 정의된 controller list에서 바꿔서 사용할 수 있다. 특히, `ur_controllers`로 구분되어 있는 controller들이 있는데, 이는 custom하게 만들어서 사용된 controller라고 보면 된다.

```yaml
controller_manager:
  ros__parameters:

    update_rate: 100 # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    io_and_status_controller:
      type: ur_controllers/GPIOController

    speed_scaling_state_broadcaster:
      type: ur_controllers/SpeedScalingStateBroadcaster

    force_torque_sensor_broadcaster:
      type: ur_controllers/ForceTorqueStateBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    scaled_joint_trajectory_controller:
      type: ur_controllers/ScaledJointTrajectoryController

    forward_velocity_controller:
      type: velocity_controllers/JointGroupVelocityController

    forward_position_controller:
      type: position_controllers/JointGroupPositionController


speed_scaling_state_broadcaster:
  ros__parameters:
    state_publish_rate: 100.0


force_torque_sensor_broadcaster:
  ros__parameters:
    sensor_name: tcp_fts_sensor
    state_interface_names:
      - force.x
      - force.y
      - force.z
      - torque.x
      - torque.y
      - torque.z
    frame_id: tool0
    topic_name: ft_data


joint_trajectory_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    state_publish_rate: 100.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
    constraints:
      stopped_velocity_tolerance: 0.2
      goal_time: 0.0
      shoulder_pan_joint: { trajectory: 0.2, goal: 0.1 }
      shoulder_lift_joint: { trajectory: 0.2, goal: 0.1 }
      elbow_joint: { trajectory: 0.2, goal: 0.1 }
      wrist_1_joint: { trajectory: 0.2, goal: 0.1 }
      wrist_2_joint: { trajectory: 0.2, goal: 0.1 }
      wrist_3_joint: { trajectory: 0.2, goal: 0.1 }


scaled_joint_trajectory_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    state_publish_rate: 100.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
    constraints:
      stopped_velocity_tolerance: 0.2
      goal_time: 0.0
      shoulder_pan_joint: { trajectory: 0.2, goal: 0.1 }
      shoulder_lift_joint: { trajectory: 0.2, goal: 0.1 }
      elbow_joint: { trajectory: 0.2, goal: 0.1 }
      wrist_1_joint: { trajectory: 0.2, goal: 0.1 }
      wrist_2_joint: { trajectory: 0.2, goal: 0.1 }
      wrist_3_joint: { trajectory: 0.2, goal: 0.1 }

forward_velocity_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    interface_name: velocity

forward_position_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
```

시뮬레이션을 실행하고, topic 리스트를 보면 `/joint_trajectory_controller/joint_trajectory` 토픽이 있는데, `joint_trajectory_controller`가 컨트롤러로 사용되어 있음을 알 수 있고, 지난 포스팅에서 joint controller 명령을 내렸던 것처럼 joint 명령을 내릴 수 있다.

다음의 명령을 내리면 현재 사용되는 controller의 리스트를 볼 수 있다.

```bash
ros2 control list_controllers
```

그 결과는 다음과 같을 것이다.

```bash
joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active    
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active  
```

이제 `ur_controllers.yaml`에 정의된 controller 중 하나인 `forward_position_controller`를 로드해서 controller를 바꿔서 사용할 것이다.

```bash
ros2 control load_controller forward_position_controller
```

위 명령을 사용하면 성공적으로 controller를 로드했다고 나올 것인데, 이는 `ur_controllers.yaml`처럼 미리 리스트를 등록한 경우에만 로드할 수 있다.

다시 list를 확인해보면, 다음처럼 `fowrard_position_controller`가 추가된 것을 볼 수 있다.

```bash
joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active    
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active    
forward_position_controller[position_controllers/JointGroupPositionController] unconfigured
```

그러나 `forward_position_controller`는 아직 `unconfigured` 상태이기 때문에 `configure` 상태로 만들기 위해 다음의 명령을 사용한다.

```bash
ros2 control set_controller_state forward_position_controller configure
```

그러면 list에서 `forward_position_controller`가 `inactive` 상태로 바뀐 것을 확인할 수 있다.

```bash
joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active    
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active    
forward_position_controller[position_controllers/JointGroupPositionController] inactive
```

이제 `forward_position_controller`를 `active`로 바꾸고, `joint_trajectory_controller`를 `inactive`로 바꾸면 controller를 스위칭할 수 있다.

```bash
# set_controller_state 이용
ros2 control set_controller_state joint_trajectory_controller stop
ros2 control set_controller_state forward_position_controller start

# 또는 switch_controller 이용
ros2 control switch_controllers --start forward_position_controller --stop joint_trajectory_controller
```

그러면 controller list는 다음과 같이 바뀐 것을 확인할 수 있다.

```bash
joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] inactive  
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active    
forward_position_controller[position_controllers/JointGroupPositionController] inactive
```

이제 바뀐 `forward_position_controller`를 이용하여 다음과 같이 joint control을 할 수 있게 된다.

```bash
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "layout:
  dim: []
  data_offset: 0
data: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]" --once
```

## 참고문헌
* [ros2_control documentation](https://control.ros.org/master/index.html)
* [ros2_control github](https://github.com/ros-controls/ros2_control)
* [ros2_controllers github](https://github.com/ros-controls/ros2_controllers)