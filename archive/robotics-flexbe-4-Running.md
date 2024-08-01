---
layout: note_page
title: FlexBe - Running
tags: [FlexBE]
category: "Robotics"
---

## 1. FlexBE의 behavior를 실행하는 두 가지 방법

1. FlexBE만을 이용하여 로봇을 제어하는 경우
2. 상위 제어기에 FlexBE가 포함되어 실행하는 방법

## 2. FlexBE 만을 이용한 로봇 제어

전체 시스템이 실행되고, FlexBE를 최상위 제어 인스턴스로 사용하려는 경우

- onboard behavior engine을 launch

```bash
roslaunch flexbe_onboard behavior_onboard.launch
```

- ROS master가 연결된 상태에서 [Example Behavior] 이름의 behavior 실행

```bash
rosrun flexbe_widget be_launcher -b 'Example Behavior'
```

- 해당 내용을 launch 파일에 포함시키는 경우

```xml
<arg name="behavior_name" default="Example Behavior" />
<include file="$(find flexbe_onboard)/launch/behavior_onboard.launch" />
<node name="behavior_launcher" pkg="flexbe_widget" type="be_launcher" output="screen" args="-b '$(arg behavior_name)'" />
```

## 3. 상위 제어기에 FlexBE가 포함되어 실행

action call을 통해 behavior를 실행시킬 수 있는데, FlexBE 동작을 다른 상위 제어 인스턴스에 포함하는 경우에 사용된다.

- onboard behavior engine을 launch

```bash
roslaunch flexbe_onboard behavior_onboard.launch
```

- FlexBE action server 실행 : 이 action server는 action topic인 /flexbe/execute_behavior (flexbe_msgs/BehaviorExecution)를 받는다.

```bash
rosrun flexbe_widget be_action_server
```

- 해당 내용을 launch 파일에 포함시키고, 상위 제어기에서 behavior를 실행시키는 방법

    - launch file :
    ```xml
    <include file="$(find flexbe_onboard)/launch/behavior_onboard.launch" />
    <node name="be_action_server" pkg="flexbe_widget" type="be_action_server" output="screen" respawn="true" />
    ```
    - 상위 제어기(action 요청) :
    ```python
    # action client 생성
    self._action_client = actionlib.SimpleActionClient('flexbe/execute_behavior', BehaviorExecutionAction)

    # action goal 생성
    self._action_goal = BehaviorExecutionGoal(behavior_name="Example Behavior")

    # behavior 실행
    self._action_client.send_goal(self._action_goal)
    ```

## 4. 사용자 인터페이스를 이용

이 경우는 behavior 동작을 모니터링하거나, 명령 전송, 런타임 수정 등에 사용된다.

```bash
roslaunch flexbe_widget behavior_ocs.launch
```

## 참고문헌

- [flexbe/Tutorials/Running Behaviors Without Operator - ROS Wiki](http://wiki.ros.org/flexbe/Tutorials/Running%20Behaviors%20Without%20Operator)
- [team-vigir/flexbe_behavior_engine](https://github.com/team-vigir/flexbe_behavior_engine/blob/master/flexbe_msgs/action/BehaviorExecution.action)