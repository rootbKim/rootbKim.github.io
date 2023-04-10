---
layout: post
title: Robot State Publisher 이해하기
feature-img: "assets/img/portfolio/openai.png"
thumbnail: "assets/img/portfolio/openai.png"
tags: [Gym ROS]
---

Gym 프로젝트에 사용되는 로봇의 robot state publisher 튜토리얼을 통해 robot state publisher의 동작을 이해한다.

## 1. robot_state_publisher 패키지란?

robot_state_publisher는 로봇의 상태 정보를 제공하는 패키지이다. 이 패키지는 `URDF`(Unified Robot Description Format) 파일을 읽어서 로봇의 구조와 다양한 조인트(joint)들의 상태 정보를 파악하고, 이 정보를 기반으로 tf2(Transform Library) 메시지를 생성한다. 그리고 `joint_states` 토픽([`sensor_msgs/msg/JointState`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/JointState.html))을 subscribe한다. 이 토픽은 URDF에 정의된 조인트의 상태정보를 업데이트하며, 이 업데이트 받은 조인트 값을 tf2에 반영한다.

> 이러한 tf2 메시지는 로봇의 다양한 부분들이 상대적으로 어떻게 위치하고 움직이는지에 대한 정보를 담고 있다. 이는 다른 ROS2 패키지에서 로봇을 제어하거나 센싱하는 등의 작업에서 매우 중요한 역할을 한다.

robot_state_publisher는 fixed와 movable 두 가지 타입의 클래스를 사용한다. fixed joint는 `/tf_static` 토픽에 시작 시 발행되고, movable joint는 `/tf` 토픽에 `joint_states`가 업데이트 될 때마다 게시된다.

> robot_state_publisher는 일반적으로 시뮬레이션 환경에서 사용되며, Gazebo와 같은 시뮬레이터와 함께 사용될 수 있고, 또한 실제 하드웨어에서도 사용할 수 있다.

### 1.1 Published Topics

* `robot_description(std_msgs/msg/String)`: URDF에 대한 설명을 string으로 게시한다.
* `tf(tf2_msgs/msg/TFMessage)`: movable joint에 해당하는 tf
* `tf_static(tf2_msgs/msg/TFMessage)`: static joint에 해당하는 tf

### 1.2 Subscribed Topics

* `joint_states(sensor_msgs/msg/JointState)`: 해당 토픽을 subscribe하여 로봇 포즈를 업데이트 하고, 업데이트된 `tf`를 게시한다.

### 1.3 Parameters

* `robot_description(string)`: URDF에 대한 설명으로, robot_state_publisher가 실행될 때 설정되어야 한다. 이 파라미터가 업데이트 될 때 `robot_description` 토픽에 반영된다.
* `publish_frequency(double)`: `/tf` 토픽이 발행될 수 있는 최대 주파수. 디폴트 값은 20hz
* `ignore_timestamp(bool)`: 타임스탬프에 관계없이 모든 joint states를 사용할지(true), 또는 마지막으로 게시된 것보다 최신의 상태만 업데이트할 것인지 설정(false). 디폴트 값음 false.
* `frame_prefix(string)`: `tf2`에 추가하는 임의의 prefix. 디폴트는 빈 문자열.

## 2. robot_state_publisher 튜토리얼

[Using URDF with robot_state_publisher](https://docs.ros.org/en/foxy/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html)의 튜토리얼을 기반한다.

### 2.1 Create Package

`second_ros2_ws`를 만들고, `urdf_tutorial_r2d2` 패키지를 생성한다.

```bash
mkdir -p ~/second_ros2_ws/src  # change as needed
cd ~/second_ros2_ws/src
source /opt/ros/galactic/setup.bash
ros2 pkg create urdf_tutorial_r2d2 --build-type ament_python --dependencies rclpy
cd urdf_tutorial_r2d2
```

### 2.2 URDF 파일 설치

패키지 내에 urdf 폴더를 만들고 [URDF](https://docs.ros.org/en/foxy/_downloads/872802005223ffdb75b1ab7b25ad445b/r2d2.urdf.xml)을 `r2d2.urdf.xml` 이름으로 저장하고, [RVIZ CONFIG](https://docs.ros.org/en/foxy/_downloads/96d68aef72c4f27f32af5961ef48c475/r2d2.rviz) 파일도 `r2d2.rviz` 이름으로 저장한다.

```bash
mkdir -p urdf
```

### 2.3 Publisher 작성

로봇의 현재 상태를 특정하는 방법을 지정해야 하는데, 이를 위해 로봇의 세 개의 조인트와 `odom`을 정의해야 한다.

`~/second_ros2_ws/src/urdf_tutorial_r2d2/urdf_tutorial_r2d2/state_publisher.py`를 다음과 같이 작성한다.

```python
from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        degree = pi / 180.0
        loop_rate = self.create_rate(30)

        # robot state
        tilt = 0.
        tinc = degree
        swivel = 0.
        angle = 0.
        height = 0.
        hinc = 0.005

        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'axis'
        joint_state = JointState()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['swivel', 'tilt', 'periscope']
                joint_state.position = [swivel, tilt, height]

                # update transform
                # (moving in a circle with radius=2)
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = cos(angle)*2
                odom_trans.transform.translation.y = sin(angle)*2
                odom_trans.transform.translation.z = 0.7
                odom_trans.transform.rotation = \
                    euler_to_quaternion(0, 0, angle + pi/2) # roll,pitch,yaw

                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                self.broadcaster.sendTransform(odom_trans)

                # Create new robot state
                tilt += tinc
                if tilt < -0.5 or tilt > 0.0:
                    tinc *= -1
                height += hinc
                if height > 0.2 or height < 0.0:
                    hinc *= -1
                swivel += degree
                angle += degree/4

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()
```

### 2.4 Launch

튜토리얼을 실행하기 위한 launch 파일 `~/second_ros2_ws/src/urdf_tutorial_r2d2/launch/demo.launch.py`을 다음과 같이 작성한다.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'r2d2.urdf.xml'
    urdf = os.path.join(
        get_package_share_directory('urdf_tutorial_r2d2'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='urdf_tutorial_r2d2',
            executable='state_publisher',
            name='state_publisher',
            output='screen'),
    ])
```

### 2.5 setup.py 작성

파이썬 ROS2 패키지 빌드를 위한 설정파일 `~/second_ros2_ws/src/urdf_tutorial_r2d2/setup.py`를 수정한다.

import 추가

```python
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages
```

data file 추가

```python
data_files=[
  ...
  (os.path.join('share', package_name), glob('launch/*.py')),
  (os.path.join('share', package_name), glob('urdf/*'))
],
```

`entry_point`에 추가

```python
'console_scripts': [
    'state_publisher = urdf_tutorial_r2d2.state_publisher:main'
],
```

### 2.6 빌드

```bash
cd ~/second_ros2_ws
colcon build --symlink-install --packages-select urdf_tutorial_r2d2
source install/setup.bash
```

### 2.7 실행 및 결과

런치

```bash
ros2 launch urdf_tutorial_r2d2 demo.launch.py
```

RVIZ

```bash
rviz2 -d ~/second_ros2_ws/install/urdf_tutorial_r2d2/share/urdf_tutorial_r2d2/r2d2.rviz
```

## 참고문헌

- [Using URDF with robot_state_publisher](https://docs.ros.org/en/foxy/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html)
- [github - ros/robot_state_publisher](https://github.com/ros/robot_state_publisher)