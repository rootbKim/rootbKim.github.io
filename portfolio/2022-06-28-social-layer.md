---
layout: portfolio
title: 로봇 간 회피주행을 위한 멀티로봇 아키텍처 및 Social Costmap Layer 개발
feature-img: "/assets/img/portfolio/social/social.png"
img: "/assets/img/portfolio/social/social.png"
date: 28 June 2022
excerpt: 로봇 간 회피를 위하여 navigation이 상대 로봇의 이동 의도를 예측할 수 있도록 하는 Social Layer의 개념을 제시하고 개발한 내용이다.
---

# 개요

같은 공간에서의 로봇 간의 회피를 위하여, 상대 로봇의 위치와 이동 경로를 고려한 전역 경로 생성을 할 수 있도록 하는 Navigation의 global costmap layer인 Social Layer를 개발하고, 이를 이용하여 로봇 간의 회피 기능을 검증하였다.

# 관련 링크

* 논문
  - Jihyeon Kwon, Jiyong Kim, "Avoidance Driving Method Through Costmap Reflection Using Path Information Between Autonomous Driving Robots," The 7th International Conference on Advanced Engineering – Theory and Applications, 2022.

# 프로젝트 기간

2022-06-28 ~ 2022-12-09

# 개발 참여 인원 / 담당 역할

2명 / 기능 개발 및 검증

# 기술 스택

- Multi Robots
- ROS2(foxy)
- Nav2

# 세부 내용

### Social Layer

* ROS에서는 다음과 같은 Social Layer에 대한 내용을 제공한다. 
  * [ROS social_navigation_layer](http://wiki.ros.org/social_navigation_layers)
  * [navigation_layers](https://github.com/DLu/navigation_layers)
  * [Social Robot Navigation](https://www.ri.cmu.edu/pub_files/2010/5/rk_thesis.pdf)

* 위에서 사용된 ROS Social Layer는 로봇 간의 회피를 위해 사용된 것이 아니라, 사람의 정보를 기반으로 사람을 회피하기 위한 방법으로 사용되었다.
* 다음과 같이 사람이 움직이거나, 서 있을 때의 cost를 정규분포를 이용하여 확률적으로 나타낸 것이다.

<img src="/assets/img/portfolio/social/ros_social_layer.png">

### Social Layer 전략

* ROS의 Navigation에서 동적 장애물을 회피하기 위해서는 로봇의 경로 상에 장애물이 있음을 판단하고, 해당 경로를 우회할 수 있는 전역 경로 또는 지역 경로를 재생성 해야 한다.
* Social Layer는 로봇의 global costmap에 상대 로봇의 위치와 이동 경로에 대한 cost를 부여하여, 로봇이 상대 로봇의 cost를 우회하도록 전역 경로를 재생성 하도록 한다.
* global costmap layer는 다음과 같이 기본 costmap layer에 Social Layer(여기서는 multi-robot-layer)를 사용한다.

<img src="/assets/img/portfolio/social/layer.png">

* Social Layer의 cost를 부여하기 위한 계산식은 기존의 컨셉과 동일하게 정규분포를 이용하여 위치에 대한 로봇의 존재 확률을 cost로 부여했다.

<img src="/assets/img/portfolio/social/gaussian.png">

* 그리고 로봇의 경로에 cost를 부여하였는데, 이는 로봇으로부터 먼 지점일 수록 cost의 크기를 작게 하여, 다른 로봇의 이동 시 해당 cost 때문에 주행을 방해받지 않도록 조절하였다.

<img src="/assets/img/portfolio/social/cost_distribution.png">

### 로봇 간의 통신 전략

* 상대 로봇의 위치와 이동경로를 자신의 global costmap에 부여하기 위해서는 상대 로봇과 통신할 수 있어야 한다.
* 이를 위하여 같은 AP 환경 안에서, 로봇 간에 ROS2 DDS 통신을 기반으로 통신하였고, 로봇마다 고유의 ID를 부여하여, namespace를 이용한 메시지 구분을 하였다.
* 로봇에 설치된 모듈은 다음과 같다.

<img src="/assets/img/portfolio/social/communication.png">

### 시뮬레이션 테스트 및 검증

* GAZEBO 환경에서 두 대의 로봇의 회피 상황을 Social Layer를 사용했을 때와 안했을 때의 상황에 대하여 비교하였다. 

<img src="/assets/img/portfolio/social/simulation.png">

* 테스트 결과 Social Layer를 사용했을 때, 서로의 경로가 겹치지 않고 잘 우회하는 것을 볼 수 있었고, Social Layer를 사용하지 않은 경우에는 로봇이 서로 장애물로만 판단하고, 서로 회피하려다가 충돌하는 경우도 발생했다.

<img src="/assets/img/portfolio/social/result_l.png">
<img src="/assets/img/portfolio/social/result_u.png">

### Conclusion

* 연구된 Social Layer 기법을 이용하여 2022 AETA 학회에서 논문을 발표하였다.
* 이 기법은 로봇 간의 회피를 위한 기본적인 방법으로 사용될 수 있을 만큼 효과를 가지고 있다.
* 이 Social Layer는 향후에 로봇 간의 회피를 하는데 사용되는 기본적인 방법으로 사용되었다.
* 여기서는 우선순위에 대한 개념이 없었기 때문에, 한계점도 있었는데, 향후에 기술적인 내용들이 더 추가되며 로봇 간의 회피를 잘 할 수 있도록 구현할 수 있었다.
* Navigation 스택을 활용할 수 있었다.