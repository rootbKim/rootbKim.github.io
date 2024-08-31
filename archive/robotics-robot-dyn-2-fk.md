---
layout: archive
title: 로봇동역학 - Forward Kinematics
tags: [Robot Dynamics]
category: "Robotics"
---

이전에 정리한 Homogeneous 변환 행렬을 이용하여 로봇의 각 조인트 변수가 주어졌을 때, 글로벌 좌표계에 대한 End Effector의 3차원 공간에서의 위치 및 방향을 구하는 Forward Kinematics(FK)에 대해서 정리한다.

# 1. Forward Kinematics

Forward Kinematics(FK)는 로봇의 각 joint variable이 주어졌을 때의 End-Effector의 위치와 방향을 계산하는 방식을 말한다. 여기서 joint variable은 두 개의 링크를 연결하는 revolute 또는 prismatic joint의 각도 또는 길이 값이다.

FK를 구하기 위해서 전제되는 사항은 다음과 같다.

- 로봇은 `n`개의 joint와 ground를 포함한 `n+1`개의 link를 가진다.
- 즉 joint number는 `1~n`이며, link number는 `0~n`이고, `0`은 ground 링크가 된다.
- joint `i`는 link `i-1`과 `i`를 연결하며, link `i-1`에 고정된다.
- joint `i`는 다음과 같이 joint variable $$q_i$$을 가진다.
$$
q_i = \theta_i(joint \, i \, is \, revolute) \; \; or \; \; d_i(joint \, i \, is \, prismatic)
$$
- frame `i`는 link `i`에 고정된다. 즉, link `i`가 움직이면, frame `i`도 움직인다.
- link `0`은 ground이며, 고정된 좌표계가 된다.

# 2. Homogeneous Transformation

앞서 정리한 Homogeneous 변환 행렬을 $$A_i$$라고 표현하고, 이는 $$o_{i-1}x_{i-1}y_{i-1}z_{i-1}$$ 좌표계에 대한 $$o_{i}x_{i}y_{i}z_{i}$$ 좌표계의 Homogeneous 변환 행렬을 뜻한다.

로봇에서 이 Homogeneous 변환 행렬은 로봇의 각 joint의 joint variable $$q_i$$에 의해 결정된다.

$$
A_i = A_i(q_i)
$$

만약 $$o_{i}x_{i}y_{i}z_{i}$$ 좌표계(i 링크)에 대한 $$o_{j}x_{j}y_{j}z_{j}$$ 좌표계(j 링크)의 Homogeneous 변환 행렬 $$T^i_j$$을 나타내면 다음과 같이 나타낼 수 있다.

$$
T^i_j = A_{i+1}A_{i+2} \dots A_{j-1}A_{j}
$$

로봇에서 베이스 프레임(0번)에서 n번 링크의 프레임(n번)의 Homogeneous 변환 행렬을 나타내면 다음과 같이 된다.

$$
H = \begin{bmatrix}R^0_n & O^0_n \\ 0 & 1 \\ \end{bmatrix} = T^0_n = A_1(q_1) \dots A_n(q_n)
$$

여기서 각 Homogeneous 변환 행렬은 다음과 같다.

$$
A_i = \begin{bmatrix}R^{i-1}_i & O^{i-1}_i \\ 0 & 1 \\ \end{bmatrix}
$$

이 0번 프레임에서 End-Effector 프레임인 n번 프레임까지의 Homogenous 회전 행렬을 구하면 이게 바로 Forward Kinematics가 된다. 즉, 기준 좌표계에 대한 End-Effector의 position 및 orientation을 구할 수 있게 된 것이다!

# 3. Denavit-Hartenberg(D-H) Convention

위에서 Homogeneous 변환 행렬을 이용하여 3차원 공간에서의 해를 구하려면 6개의 자유도를 고려해야 한다. 이를 계산하는 것은 쉬운 일이 아니다. 이러한 문제를 해결하기 위해 Denavit-Hartenberg(D-H) Convention을 이용한다. D-H convention은 각 Frame의 좌표계를 잘 설정하여, 6개의 자유도를 4개의 자유도만 고려하도록 하여 계산하는 방법을 제안한다.

D-H Convention은 4 개의 기본 변환 행렬을 이용하여 계산된다.

$$
A_i = Rot_{z,\theta_i}Trans_{z,d_i}Trans_{x,a_i}Rot_{x,\alpha_i} \\
= \begin{bmatrix} c_{\theta_i} & -s_{\theta_i} & 0 & 0 \\ s_{\theta_i} & c_{\theta_i} & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & d_i \\ 0 & 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} 1 & 0 & 0 & a_i \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & c_{\alpha_i} & -s_{\alpha_i} & 0 \\ 0 & s_{\alpha_i} & c_{\alpha_i} & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix} \\
= \begin{bmatrix} c_{\theta_i} & -s_{\theta_i}c_{\alpha_i} & s_{\theta_i}s_{\alpha_i} & a_ic_{\theta_i} \\ s_{\theta_i} & c_{\theta_i}c_{\alpha_i} & -c_{\theta_i}s_{\alpha_i} & a_is_{\theta_i} \\ 0 & s_{\alpha_i} & c_{\alpha_i} & d_i \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

여기서 변수는 4개($$\theta_i, d_i, a_i, \alpha_i$$)로 보이나, 이 중 3개는 상수이고, 1개만 변수가 된다. 즉 이 변수가 그 조인트의 회전 또는 선형 변환 값이다. 만약 조인트가 revolute 조인트이면 $$\theta_i$$가 변수가 되고, prismatic 조인트이면 $$d_i$$가 변수가 된다.

> 만약 변수가 한 개도 없다면, 해당 프레임 사이에는 조인트가 없는 경우이다.

그럼 이 D-H Convention을 사용하기 위해서 Frame 좌표계를 잘 설정하는 것이 중요한데 그 좌표계 설정 규칙은 다음과 같다.

- DH1. $$x_i$$ 축과 $$z_0$$ 축은 수직이다.
- DH2. $$x_i$$ 축과 $$z_0$$ 축은 교차한다.

위 두 가지의 규칙이 성립하면 4 개의 변수는 다음과 같이 정의된다.

- $$a_i$$ : $$x_i$$축 방향 $$o_i$$와 $$o_{i-1}$$ 사이의 거리(상수)
- $$\alpha_i$$ : $$x_i$$축을 축으로 $$z_{i-1}$$ 축을 기준으로 $$z_i$$ 축 사이의 각도(상수)
- $$d_i$$ : $$z_{i-1}$$축 방향 $$o_i$$와 $$o_{i-1}$$ 사이의 거리
- $$\theta_i$$ : $$z_{i-1}$$축을 축으로 $$x_{i-1}$$ 축을 기준으로 $$x_i$$ 축 사이의 각도

<img src="/assets/img/posts/230325_dh_convention.png">

이 규칙을 만족하도록 좌표계를 잡고, 각각의 조인트에 대한 변환 행렬을 잡은 후, 0번 조인트부터 n번 조인트까지의 변환 행렬을 곱하여 $$T^0_n$$을 구하면 최종적인 FK 식이 된다. 이 때, 0번 좌표계와 n번 좌표계는 동일한 방향으로 잡아야하는데, 일반적으로 0번 좌표계에 대한 end-effector의 위치를 계산하기 때문이다.

# 4. D-H Convention을 이용한 FK 계산 예제

## Planar Elbow Manipulator

<img src="/assets/img/posts/230325_dh_convention_example1.png">

|Link|$$a_i$$|$$\alpha_i$$|$$d_i$$|$$\theta_i$$|
|-|-|-|-|-|
|1|$$a_1$$|0|0|$$\theta^*_1$$|
|2|$$a_2$$|0|0|$$\theta^*_2$$|

$$
A_1 = \begin{bmatrix} c_1 & -s_1 & 0 & a_1c_1 \\ s_1 & c_1 & 0 & a_1s_1 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix} \\
A_2 = \begin{bmatrix} c_2 & -s_2 & 0 & a_2c_2 \\ s_2 & c_2 & 0 & a_2s_2 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

> `*` 으로 표시된 항목이 변수임을 나타낸다.

> 여기서 $$c_{12} = cos(\theta_1 + \theta_2)$$이며, $$s_{12}$$도 마찬가지다.

최종적으로 $$T$$를 구하면 다음과 같다.

$$
T^0_2 = A_1A_2 = \begin{bmatrix} c_{12} & -s_{12} & 0 & a_1c_1 + a_2c_{12} \\ s_{12} & c_{12} & 0 & a_1s_1 + a_2s_{12} \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

## Three-Link Cylindrical Robot

<img src="/assets/img/posts/230325_dh_convention_example2.png">

|Link|$$a_i$$|$$\alpha_i$$|$$d_i$$|$$\theta_i$$|
|-|-|-|-|-|
|1|0|0|0|$$\theta^*_1$$|
|2|0|-90|$$d^*_2$$|0|
|3|0|0|$$d^*_3$$|0|
|4|0|90|0|0|

여기서 4번 링크는 3번 좌표계를 0번 좌표계와 동일한 좌표계로 맞춰주기 위해서 제자리에서 static하게 한번 더 변환하는 경우에 추가한다.

$$
A_1 = \begin{bmatrix} c_1 & -s_1 & 0 & 0 \\ s_1 & c_1 & 0 & 0 \\ 0 & 0 & 1 & d_1 \\ 0 & 0 & 0 & 1 \end{bmatrix} \\
A_2 = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & -1 & 0 & d_2 \\ 0 & 0 & 0 & 1 \end{bmatrix} \\
A_3 = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & d_3 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

최종적으로 $$T$$를 구하면 다음과 같다.

$$
T^0_3 = A_1A_2A_3 = \begin{bmatrix} c_{12} & 0 & -s_1 & -s_1d_3 \\ s_{12} & 0 & c_1 & c_1d_3 \\ 0 & -1 & 0 & d_1+d_2 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$