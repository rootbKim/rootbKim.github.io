---
layout: post
title: MIP를 이용한 우회경로 알고리즘의 연구
feature-img: "/assets/img/portfolio/mip/mip.png"
img: "/assets/img/portfolio/mip/mip.png"
date: 10 May 2021
excerpt: 노드-링크 기반의 맵에서 Mixed Integer Programming을 이용한 우회경로 알고리즘에 대한 연구 내용이다.
---

# 개요

한 공간에서 자율주행이 가능한 여러 대의 로봇이 주행하는데 있어, 로봇 간의 교통관리 기능이 필수적이다. 로봇이 서로를 로봇이라고 인식하지 못하고, 장애물이라고 인식하기 때문에 서로 회피하려다가 충돌이 일어날 수 있고, 교착이 일어날 수 있기 때문이다. 이러한 문제를 해결하기 위하여 노드-링크 기반의 맵을 만들고, 그 맵을 수학적으로 모델링하여 MIP 알고리즘을 이용하여 경로가 겹치지 않게 경로를 지정해주는 방법에 대하여 연구하였다.

# 관련 링크

* 논문
    - [Ji-Yong Kim, Jae-Wong Cho, Ji-Hyeon Kwon, "Multi-robot traffic management using MIP path negotiation scheduler," International Conference on Control, Automation and Systems, 2021.](https://ieeexplore.ieee.org/document/9649860)

# 연구 기간

2021-05-10 ~ 2021-08-24

# 연구 참여 인원 / 담당 역할

2명 / MIP 모델 설계, OR-Tools 및 Groubi 라이브러리 이용한 계산 알고리즘 구현, 성능 검증을 위한 멀티 로봇 시뮬레이션 환경 구성

# 기술 스택

- Mixed Integer Programming
- OR-Tools
- Groubi
- ROS(noetic)
- GAZEBO
- C++
- Python

# 세부 내용

### MIP 모델 정의

* MIP 모델을 정의하기 위하 맵 모델을 다음과 같이 `arc`와 `node`로 정의하였다.

<img src="/assets/img/portfolio/mip/mip_model.png">

* 로봇 간의 경로 생성을 위해 서버에서는 각 로봇이 같은 노드를 점유하지 않도록 우회경로를 계산하기 위하여, 다음의 MIP 모델을 이용하여 최적의 경로를 계산한다.

<img src="/assets/img/portfolio/mip/mip_model_equations.png">

> MIP 모델에 대해서 간략하게 정리하면 다음과 같으며, 자세한 내용은 [논문](https://ieeexplore.ieee.org/document/9649860)을 참조한다.
>   1. Objective Function이며, 그 이후의 식들은 모두 Constraints으로, 아래의 Constratints를 모두 만족하면서 Obejctive Function을 최소화 하는 최적 해(arc list)를 찾는 것이다.
>   2. 로봇의 초기 위치 설정
>   3. 로봇이 특정 시간에 특정 arc에 있어야 함
>   4. arc에 대한 점유 설정으로, 같은 시간에 두 대 이상이 같은 arc를 점유할 수 없음
>   5. node에 대한 점유 설정으로, 같은 시간에 두 대 이상이 같은 node를 점유할 수 없음
>   6. flow conservation constraints
>   7. 상대 로봇이 각각의 시간 스탭마다 점유하고 있는 arc

* 다음은 각 변수에 대한 설명을 정리한 Table이다.

<img src="/assets/img/portfolio/mip/mip_model_variables.png">

### 실험 환경

* 다음은 위에서 구현한 MIP 모델을 검증하기 위한 테스트 환경으로, ROS noetic 버전의 GAZEBO 환경을 구성하였다.

<img src="/assets/img/portfolio/mip/gazebo.png">

* 시뮬레이션에서 사용한 AMR은 2d differential drive를 가진 모델을 사용하였으며, 자율주행은 ROS navigation 환경을 이용하였다.

* 로봇에 경로를 계산하여 명령하기 위하여 맵을 node-link 구조로 만들어, 명령을 내릴 수 있는 환경을 구성하였고, 이를 이용하여 MIP를 이용해 계산된 경로를 따라 움직이도록 환경 구성을 하였다.

* 로봇의 초기 위치에서 목적지 노드를 입력하면, 최단 경로로 계산이 되며, 로봇 간의 같은 arc를 점유하게 되는 순간을 모니터링하여, 그 순간에 MIP를 이용한 우회경로를 생성하여 둘 중 한 대의 로봇을 우회하도록 하였다.

* MIP 계산을 위하여 구글의 최적화 계산 오픈소스 라이브러리인 [OR-Tools](https://developers.google.com/optimization)를 이용했으나, 계산의 속도 등의 문제로 [GUROBI](https://www.gurobi.com/)의 gurobi-optimizer인 `Gurobi9.1.2`를 사용하여 구현하였다.

### 실험 및 결과

* 실험은 비교를 위하여 FIFO 방식을 이용한 방식과 비교를 하였는데, FIFO 방식은 단순히 먼저 node를 점유한 로봇이 지나갈 때까지 나중에 온 로봇이 대기하는 방식을 뜻한다.

* MIP를 이용했을 때의 경로 변화는 다음과 같았다.

<img src="/assets/img/portfolio/mip/mip_test.png">

* FIFO 방식과의 비교 결과, FIFO 방식보다 MIP를 이용하여 우회경로를 생성했을 때 시간이 효과적으로 단축됨을 알 수 있다.

<img src="/assets/img/portfolio/mip/mip_graph.png">

### Conclusion

* 이 연구를 통해 MIP 모델에 대해서 학습하고, 교통관리를 위한 MIP 모델을 만들어낼 수 있었다.
* MIP 모델을 적용하기 위해 구글의 OR-Tools와 Gurobi Optimizer를 이용하면서 오픈소스 라이브러리를 이용한 솔루션을 만들 수 있었다.
* 멀티 로봇에 대한 제어를 위해 필요한 아키텍처를 구성하고, 멀티 로봇 제어에 필요한 정보들을 학습할 수 있었다.
* ROS GAZEBO 환경에서의 멀티로봇 환경 구축을 할 수 있었다.
* 연구된 MIP 모델의 한계점으로, arc의 길이가 모두 동일한 경우에만 사용할 수 있었기 때문에, 실제로 적용하기 위해서는 arc의 길이가 다른 환경에서의 적용을 위한 추가적인 방법이 필요할 것으로 생각된다.