---
layout: post
title: Gait Rehabilitation Robot - Inverse Dynamics
feature-img: "assets/img/portfolio/gait.jpg"
thumbnail: "assets/img/portfolio/gait.jpg"
tags: [Gait Rehabilitation Robot]
---

역동역학 기반의 파워어시스트 보행재활로봇인 DDgo Pro의 역동역학 기반 훈련 알고리즘을 위하여 로봇의 5절 링크 메커니즘의 역동역학 해석 과정을 간단하게 다루며, 자세한 수식 과정은 생략한다.

## 1. 5절 링크 메커니즘의 Newton-Euler 해석

### 1.1 비장애인의 기준 보행 운동

비장애인의 기준 보행 운동이란 DDgo Pro의 Passive Mode 훈련에서 참조하는 크랭크 축 운동을 뜻한다. 다음은 DDgo Pro에서 사람이 2.5km/h의 속도로 보행했을 때 크랭크 축의 각도에 대한 크랭크 축의 각속도 값을 나타난다. 여기서 크랭크 축의 0도는 크랭크 암이 지면과 수평일 때를 나타낸다.

<img src="/assets/img/posts/230618_crank_angle_velocity_trajectory.jpg">

그리고 이를 시간에 대한 크랭크 축의 각도 프로파일을 수치적 방법으로 구하였고, 그 결과는 다음과 같다.

<img src="/assets/img/posts/230618_time_crank_angle.jpg">

### 1.2 각 링크의 운동 해석

크랭크 축이 0도에서 360도 까지 기준 보행 운동 프로파일을 따라 회전할 때, 각 조인트 및 각 링크의 운동을 해석하기 위하여 5절 링크의 순기구학과 역기구학을 해석하였다. 다음은 5절 링크의 각 조인트의 좌표 축을 D-H Convention에 맞게 도식한 것이다. 이를 통해 순기구학을 해석하여 각 조인트의 회전행렬을 구하였다.

<img src="/assets/img/posts/230618_five-link-mechanism.png">

다음으로 역기구학을 통해 크랭크 축의 각도에 따른 각 관절의 각도를 구하였고, 그 결과는 다음과 같다.

* Joint Angle

<img src="/assets/img/posts/230618_joint_angle.jpg">

* Joint Angular Velocity

<img src="/assets/img/posts/230618_joint_angular_velociy.jpg">

* Joint Angular Acceleration

<img src="/assets/img/posts/230618_joint_angular_acceleration.jpg">

또한 커플러와 로커의 무게중심에서의 선가속도와 각 링크의 각가속도를 구하였고, 그 결과는 다음과 같다.

* Acceleration of Coupler

<img src="/assets/img/posts/230618_acceleration_coupler.jpg">

* Acceleration of Rocker

<img src="/assets/img/posts/230618_acceleration_rocker.jpg">

* Angular Acceleration of the tree links

<img src="/assets/img/posts/230618_angular_accleration.jpg">

### 1.3 Newton-Euler 방법을 이용한 역동역학 해석

기준 보행 운동 궤적을 구현하기 위해 필요한 크랭크 축 토크를 계산하기 위하여 Newton-Euler 방법을 이용하였다. 커플러 및 로커의 운동에 대한 크랭크 축 토크를 Newton-Euler 방법을 이용하여 해석하였으며, 커플러 끝에 연결된 발판의 하중을 외력으로 가정하였다.

다음은 각 링크의 free-body diagram이다.

<img src="/assets/img/posts/230618_free_body_diagram.png">

free-body diagram에 대해 힘과 모멘트 식을 구하여, 이를 크랭크 축의 각도에 대한 각 조인트에 걸리는 힘과 토크를 구할 수 있다. 여기에 기준 보행운동 궤적을 대입하면 다음 그래프와 같이 5절 링크 메커니즘이 2.5km/h의 보행 속도로 운동하기 위해 시간에 대한 크랭크 축 토크를 구할 수 있다. 또한 크랭크 축 각도에 대한 크랭크 축의 토크 결과를 구할 수 있다.

* 시간-크랭크 축 토크

<img src="/assets/img/posts/230618_time_crank_torque.jpg">

* 크랭크 축-크랭크 축 토크

<img src="/assets/img/posts/230618_crank_torque_oneside.jpg">

DDgo Pro는 위상이 180도 차이나는 두 개의 5절 링크를 가지고 있고, 두 개의 크랭크 축은 하나의 축에 의해 움직인다. 결과적으로 양 쪽 링크를 고려한 토크 프로파일을 다음과 같이 구할 수 있다.

* 두 개의 크랭크 축을 고려한 크랭크 축-크랭크 축 토크

<img src="/assets/img/posts/230618_crank_torque.jpg">

## 2. 천이 구간 보상 토크 프로파일

스윙 발의 힐스트라이크 지점과 지지 발의 toe-off 지점에서 양 발판이 공중에 떠있는 짧은 천이 구간(천이 구간은 크랭크 축 각도를 기준으로 165도와 345도를 중심으로 형성)이 발생하게 된다. 그러면 발판에 가해지는 하중으로 토크가 서로 상쇄되어, 보행을 유지하려면 크랭크 축에 큰 토크가 필요하게 된다. 따라서 천이 구간에서 보행 속도를 유지시켜 주기 위하여 속도에 비례한 천이 구간 보상 토크 프로파일을 다음과 같이 구현하였다.

<img src="/assets/img/posts/230618_transition.jpg">

## 3. 체중 보상 토크 프로파일

사람의 무게가 발판에 실렸을 때 사람의 무게를 보상하여 주어진 운동을 수행할 수 있도록 추가적인 크랭크 축의 토크를 계산하였다. 발판에 힘 센서가 없기 때문에 발판에 실리는 힘을 측정할 수 없어서, 보행 과정에서 각 발판에 수직 방향으로 사람의 스윙 다리의 무게가 실린다고 가정하였다. 이 무게를 실험적으로 가정하여 체중의 5%로 가정하고, Jacobian을 이용하여 크랭크 축에 대한 체중 보상 토크 프로파일을 다음과 같이 구현하였다.

<img src="/assets/img/posts/230618_mass_torque.jpg">
