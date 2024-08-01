---
layout: note_page
title: gym-gazebo2 분석
tags: [Gym Gazebo]
category: "Robotics"
---

gym-gazebo2의 분석을 위해서 ROS 1 기반인 gym-gazebo 관련 논문 [Extending the OpenAI Gym for robotics: a toolkit for reinforcement learning using ROS and Gazebo](https://arxiv.org/pdf/1608.05742.pdf)과 [Acutronic Robotics](https://acutronicrobotics.com/)에서 발표한 arxiv 논문 [gym-gazebo2, a toolkit for reinforcement learning using ROS 2 and Gazebo](https://arxiv.org/pdf/1903.06278.pdf)의 내용을 간략하게 정리한다.

## 1 Extending the OpenAI Gym for robotics: a toolkit for reinforcement learning using ROS and Gazebo

[OpenAI gym](https://www.gymlibrary.dev/)을 이용하여, ROS와 GAZEBO 시뮬레이션 환경에서 강화학습의 방법에 대한 내용이다.

<img src="/assets/img/posts/230304_gym_gazebo_architecture.png">

위의 아키텍처는 해당 논문에서 가져온 내용으로, OpenAI gym과 Gazebo 환경을 ROS가 이어주는 역할을 하며, `autopilot` 이라는 모듈을 이용해 시뮬레이션 환경에서 로봇의 구동을 구현하지만, 이는 시뮬레이션 환경에서의 구동을 위한 모듈로 선택사항으로 제시된다.

해당 논문에서는 Turtlebot을 이용하며, 특정한 맵에서 Lidar 정보만을 이용하여 벽과 충돌하지 않고 주행하는 강화학습에 대해서 다룬다.

강화학습에 사용된 알고리즘은 `Q-Learning`이라는 방법과 `Sarsa`라는 방법을 각각 이용하며, 두 가지 방식으로 학습시킨 결과를 비교한다. 저자는 두 가지 알고리즘에 대한 비교는 다음 사이트를 참조한다. Q-Learning과 Sarsa에 대한 비교는 따로 다루기로 한다.

* [Reinforcement Learning: Q-Learning vs Sarsa.](http://www.cse.unsw.edu.au/~cs9417ml/RL1/algorithms.html)
* [Reinforcement Learning: Sarsa vs Qlearn](https://studywolf.wordpress.com/2013/07/01/reinforcement-learning-sarsa-vs-q-learning/)

로봇이 수행하는 `Action`은 Forward v = 0.3m/s 와 Turn(Left/Rigth) w = 0.05m/s, w = $\pm$0.3rad/s이며, `Rewards`는 Forward: 5, Turn: 1, Crash: -200으로 설정하고 실험을 진행한다.

이제 Q-Learning과 Sarsa 두 가지 방법으로 강화학습을 수행하는데, Q-Learning과 Sarsa의 간단한 코드는 다음과 같다.

<img src="/assets/img/posts/230304_gym_gazebo_qlearning.png">
<img src="/assets/img/posts/230304_gym_gazebo_sarsa.png">

학습 시 사용되는 파라미터의 내용은 아래와 같으며, 논문에서는 alp = 0.2, gam = 0.9, eps = 0.9 값을 사용했다.

<img src="/assets/img/posts/230304_gym_gazebo_learning_parameters.png">

총 3000 번의 에피소드를 시뮬레이션 하며, 각 에피소드에서는 최대 1500 번의 반복을 가진다. 각 반복에서 하나의 액션을 `observation`으로 선택하며, 다음 반복에서 학습하는데 사용된다. Q-Learning과 Srasa의 큰 차이점으로 이 하나의 `observation`을 선택하는 방식에 있는 것 같다.

두 강화학습의 결과는 Q-Learning의 방식이 Sarsa 보다 더 빠르게 학습할 수 있지만, Q-Learning이 Sarsa보다 움직임이 더 위험한 움직임(벽에 붙어서 주행)을 보여주고, Sarsa는 조금 더 부드러운 움직임을 보여준다는 것으로 정리된다.

## 2 gym-gazebo2, a toolkit for reinforcement learning using ROS 2 and Gazebo

gym-gazebo의 업그레이드 버전으로, gym-gazebo와 마찬가지로 OpenAI의 Gym 모듈을 사용한다.

gym-gazebo의 설치 복잡성이나 많은 유저들이 사용하는데 어려움을 겪었기 때문에, 기존의 gym-gazebo를 버전업한 것이 아닌 새로운 라이브러리를 개발한 것이 gym-gazebo2인데, 이는 gym-gazebo2가 보다 실행하는데 쉽고, 로봇의 특정 아키텍처를 수정하거나, 자신의 로봇을 추가하는 등 편의성을 증가시킨다. gym-gazebo2는 ros2의 python client 라이브러리를 이용해서 개발되었다.

gym-gazebo2는 gym-gazebo2, ros2, gazebo로 구성되어있다. gym-gazebo2 모듈은 환경 생성과 OpenAI Gym에 등록하는 역할을 하는데, Gym을 라이브러리 형태로 가져와서 사용한다.gym-gazebo에서는 Gym을 라이브러리가 아닌 Gym의 extension 형태로 개발되었는데, 라이브러리로 가져옴으로써 유연성이 높아지는 장점이 있다. ROS2는 gym-gazebo와 가상 시뮬레이션 환경인 gazebo의 로봇 또는 실제 환경의 로봇을 이어주는 미들웨어 역할을 한다.

ROS2를 이용한 기본 기능은 다음과 같은 세 가지로 구분된다.
- init: 클래스 생성자로서, 환경을 초기화는 기능이다. 
- step: 하나의 action을 수행하는 것으로, action이 실행되면 실행된 결과에 대한 reward를 리턴하고, action이 성공했는지, episode가 끝났는지 여부를 리턴한다.
- reset: 초기 상태로 리셋하는 기능이다.

gym-gazebo2는 코드 형상 관리를 위해 `/utils` 폴더 아래에 다음과 같은 유틸리티 코드들을 보관한다.
- ut_gazebo: gazebo 관련 유틸리티
- ut_generic: 다른 그룹과 관련 없는 유틸리티
- ut_launch: 환경 초기화와 관련된 유틸리티
- ut_mara: MARA 로봇 환경과 관련된 유틸리티
- ut_math: 수학적인 계산과 관련된 유틸리티

MARA를 이용한 강화학습은 총 4 가지 환경에 따라 학습을 진행한다.
1. end-effector의 위치와 목적지 간의 거리만 고려
2. 1번과 함께 end-effector의 방향도 고려
3. 1번의 상황에 충돌 발생 시 penalty 부여
4. 2번의 상황에 충돌 발생 시 penalty 부여

ROS2Learn 레포지토리에 gym-gazebo2를 이용하여 위 4 가지 환경에 대한 학습을 할 수 있는 테스트를 모아두었다. 자세한 reward 식과, 학습의 결과는 논문을 참조한다.

## 참고문헌

- [gym-gazebo arxiv 논문](https://arxiv.org/pdf/1608.05742.pdf)
- [gym-gazebo2 arxiv 논문](https://arxiv.org/pdf/1903.06278.pdf)
- [gym-gazebo2 깃헙 페이지](https://github.com/AcutronicRobotics/gym-gazebo2)
