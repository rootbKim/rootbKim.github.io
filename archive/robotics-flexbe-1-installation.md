---
layout: archive
title: FlexBE 설치
tags: [FlexBE]
category: "Robotics"
---

# 1. FlexBE 설치

```bash
cd ~/catkin_ws/src

# CLONE
git clone https://github.com/team-vigir/flexbe_behavior_engine.git
git clone https://github.com/FlexBE/flexbe_app.git
git clone https://github.com/FlexBE/generic_flexbe_states.git

# BUILD
cd ~/catkin_ws
catkin_make
```

# 2. Behavior Repository 생성

```bash
# your_project_name_behaviors 이름으로 repository 생성
# repo 생성 시 생성 위치가 src 폴더 아래에 잘 생성되있는지 확인 필요
cd ~/catkin_ws/src
rosrun flexbe_widget create_repo [your_project_name]

# BUILD
cd ~/catkin_ws
catkin_make
```

설치 시 다음과 같은 repository가 자동으로 생성된다.

<img src="/assets/img/posts/240202_flexbe_repository.png">

# 3. FlexBe App 실행

```bash
rosrun flexbe_app run_app
```

- FlexBE APP 실행화면

<img src="/assets/img/posts/240202_flexbe_launch.png">

FlexBE App 최초 실행 시 위에서 생성한 behavior repository가 나타나지 않는다면, [Configuration]-[Workspace]-[Force Discover]를 누르면 현재 workspace에 생성되어 있는 behavior repo를 찾아온다.

- FlexBE의 [Configuration] 탭 화면

<img src="/assets/img/posts/240202_flexbe_configuration.png">

- Workspace의 Force Discover 버튼을 누르면 다음과 같이 behavior repo를 가져옴

<img src="/assets/img/posts/240202_flexbe_discover.png">

# 참고문헌

- [FlexBE](http://philserver.bplaced.net/fbe/)
- [ros wiki - flexbe](http://wiki.ros.org/flexbe)
- [team-vigir/flexbe_behavior_engine](https://github.com/team-vigir/flexbe_behavior_engine)
- [FlexBE/flexbe_app](https://github.com/FlexBE/flexbe_app)
- [FlexBE/generic_flexbe_states](https://github.com/FlexBE/generic_flexbe_states)
- [team-vigir/vigir_behaviors](https://github.com/team-vigir/vigir_behaviors/tree/master/vigir_flexbe_launch)