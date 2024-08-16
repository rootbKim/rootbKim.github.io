---
layout: note_page
title: ROS GAZEBO Spawner 이해하기
tags: [ROS2, GAZEBO]
category: "Robotics"
---

로봇 모델을 Gazebo나 RViz와 같은 시뮬레이션 환경에 로드하고 실행할 때 사용되는는 도구인 spawner에 대해 알아본다.

# 1. Spawner란?

`ROS2 spawner`는 로봇 모델을 Gazebo나 RViz와 같은 시뮬레이션 환경에 로드하고 실행할 때 사용되는 도구이다. 일반적으로 URDF 또는 SDF 형식으로 정의된 로봇 모델 파일을 사용하여 spawner를 사용하여 시뮬레이션 환경에 로봇 모델을 로드할 수 있다.

ROS2에서 spawner를 사용하기 위해서는 `gazebo_ros` 패키지와 `robot_state_publisher` 패키지가 필요하며, 또한 URDF 또는 SDF 형식으로 정의된 로봇 모델 파일이 필요하다.

# 2. Spawner 사용하기

`gazebo_ros_pkgs` 패키지의 `spawn_entity.py` 스크립트를 사용하여 Gazebo 시뮬레이터에 로봇 모델을 로드하고 실행한다. `spawn_entity.py` 스크립트는 `gazebo_msgs/msg/SpawnEntity` 메시지를 사용하여 Gazebo 시뮬레이터에 로봇 모델을 추가한다.

command line을 이용하여 실행하는 방법은 다음과 같다.

```bash
# 빈 GAZEBO 시뮬레이션 실행하기
ros2 launch gazebo_ros gazebo.launch.py

# spawn_entity.py 스크립트 실행하기
ros2 run gazebo_ros spawn_entity.py -file my_robot.urdf -entity my_robot -x 0 -y 0 -z 1
```

여기서 `my_robot.urdf`는 사용할 URDF 파일을, `my_robot`은 spawn할 모델의 이름을 나타냅니다. `-x, -y, -z` 옵션은 모델이 spawn될 위치를 나타낸다.

위의 command를 `launch` 파일로 작성하면 다음과 같다.

```python
import os


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # gazebo 실행
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Gazebo launch
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )   

    # spawn entity 실행
    spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=['-entity', 'my_robot', '-file', 'my_robot.urdf', '-x', '0', '-y', '0', '-z', '1'],
        output='screen')

    return LaunchDescription([
        gazebo_cmd,
        spawner_cmd,
    ])
```

# 참고문헌
* [ros-simulation/gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs)