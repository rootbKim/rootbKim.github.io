---
layout: note_page
title: 로봇동역학 - Velocity Kinematics
tags: [Robot Dynamics]
category: "Robotics"
---

End Effector의 선속도 및 각속도와 각 joint variable의 관계를 나타내는 Jacobian에 대해 정리한다.

## 1. Jacobian

Jacobian은 End Effector(또는 manipulator의 특정 point)의 선속도 및 각속도와 각 joint의 속도값의 관계를 나타낸다.

수학적으로 Forward Kinematics는 데카르트 좌표 공간에서의 위치와 방향과 joint 위치의 관계를 정의하였는데, 속도의 관계는 다음과 같이 Jacobian으로 결정된다.

$$
\bold{\dot{q}} = \bold{J^{-1}} \bold{\dot{x}}
$$

여기서 $$\bold{\dot{q}}$$는 각 joint의 속도이며, $$\bold{J}$$는 Jacobian Matrix, $$\bold{\dot{x}}$$는 End Effector의 선속도 및 각속도이다.

따라서 $$\bold{\dot{q}}$$과 $$\bold{\dot{x}}$$은 다음과 같은 형태가 될 것이다.

$$
\bold{\dot{q}} = \begin{bmatrix} \bold{\tilde{v}} \\ \bold{\tilde{w}} \end{bmatrix}_{6 \times 1} \\
\bold{\dot{q}} = \begin{bmatrix} \dot{q_1} \\ \dots \\ \dot{q_n} \end{bmatrix}_{n \times 1}
$$

그리고, Jacobian Matrix는 $$6 \times 1$$ 크기의 행렬이 될 것이다. 여기서 $$n$$은 manipulator의 joint 수이다.

Jacobian Matrix는 Manipulator에서 중요한 개념이다. Trajectory를 부드럽게 planning을 하거나, Singular 상황을 결정($$\det(\bold{J}) = 0$$)하거나, 동적 운동 방정식을 도출하거나, End Effector와 각 joint 간의 힘과 토크의 관계를 구하는 등에 사용된다.

## 2. 고정된 축에서의 Angular Velocity 계산의 예

고정된 축에 대한 rigid body의 움직임에서, rigid body의 모든 점은 원운동을 한다. 만약 $$\bold{k}$$가 회전 축의 방향을 나타내는 단위 벡터라면, rigid body의 각속도는 다음과 같이 계산된다.

$$
\omega = \dot{\theta} \bold{k}
$$

그리고 rigid body의 각 point의 선속도는 다음과 같이 계산된다.

$$
\bold{v} = \omega \times r
$$

여기서 $$r$$은 rigid body의 점이 축으로부터 떨어진 거리를 나타낸다.

위의 예와 같이, 원점이 고정된 상황에서 각속도는 rigid body의 각 점의 선속도를 구하는데 사용된다.

만약 원점의 프레임이 고정되어 있지 않고, 움직이거나, 회전 운동을 하는 경우에는 rigid body의 한 점의 운동을 어떻게 정의할 수 있을까? Manipulator의 운동을 정의하기 위해서는 이 질문에 대한 답이 필요하다.

## 3. Skew Symmetric Matrix

Skew symmetric matrix는 좌표계간의 상대 속도 변환을 단순화하는데 사용된다. 이러한 성질을 이용하여 고정되지 않은 좌표계 위에서의 회전 운동을 일반화하는데 사용한다.

### 3.1 Skew symmetric matrix의 정의
Skew symmetric matrix $$S$$의 정의는 다음과 같다.

$$
S^T + S = 0
$$

만약 $$3 \times 3$$ 행렬이라면, 다음과 같을 것이다.

$$
s_{ij} + s_{ji} = 0 \;\;\; i, j = 1, 2, 3
$$

즉, $$S$$는 다음과 같다.

$$
S = \begin{bmatrix} 0 & -s_3 & s_2 \\ s_3 & 0 & -s_1 \\ -s_2 & s_1 & 0 \end{bmatrix}
$$

만약 $$a = (a_x, a_y, a_z)^T $$인 3차원 백터가 있다면 $$S(a)$$ skew symmetric matrix는 다음과 같다.

$$
S(a) = \begin{bmatrix} 0 & -a_z & a_y \\ a_z & 0 & -a_x \\ -a_y & a_x & 0 \end{bmatrix}
$$

### 3.2 Skew symmetric matrix의 성질

Skew Symmetric 행렬의 성질을 정리하면 다음과 같다.

1. Skew Symmetric 행렬 연산은 선형이다. 여기서, $$\alpha$$와 $$\beta$$는 스칼라 값이며, $$a$$와 $$b$$는 $$\bold{R}^3$$ 백터이다.

$$
S(\alpha a + \beta b) = \alpha S(a) + \beta S(b)
$$

2. $$\bold{R}^3 백터인 $$a$$, $$p$$일 때 다음을 만족한다.

$$
S(a)p = a \times p
$$

3. $$R \in SO(3)$$(skew symmetric and orthogonal)이고, $$a, b \in \bold{R}^3$$일 때 다음을 만족한다.

$$
R(a \times b) = Ra \times Rb
$$

4. $$R \in SO(3)$$이고, $$a \in \bold{R}^3$$일 때 다음을 만족한다.

$$
RS(a)R^T = S(Ra)
$$

5. $$n \times n$$ skew symmetric matrix S는 $$X \in \bold{R}^n$$인 모든 백터에 대해서 다음을 만족한다.

$$
X^TSX = 0
$$

### 3.3 Rotation matrix의 도함수

모든 $$\theta$$에 대하여 $$R=R(\theta) \in SO(3)$$일 때 다음을 만족한다.

$$
R(\theta)R(\theta)^T = I
$$

양 변을 $$\theta$$에 대해 미분한다.

$$
\frac{d R}{d \theta}R(\theta)^T + R(\theta)\frac{dR^T}{d\theta} = 0
$$

이는 Skew symmetric matrix와 비슷한 형태인데, $$S$$를 다음과 같이 정의하고,

$$
S = \frac{d R}{d \theta}R(\theta)^T
$$

그리고 $$S$$의 transpose는 다음과 같다.

$$
S^T = (\frac{d R}{d \theta}R(\theta)^T)^T = R(\theta) \frac{dR^T}{d\theta}
$$

따라서 $$S + S^T = 0$$이 되므로, Skew symmetric matrix의 성질을 만족한다.

$$ S = \frac{d R}{d \theta}R(\theta)^T $$의 양 변에 $$R$$을 우측에 곱하면 다음과 같이 된다.

$$
\frac{d R}{d \theta} = SR(\theta)
$$

이는 회전 행렬 $$R$$의 도함수를 계산하는 것이 skew symmetric matrix $$S$$와 $$R$$의 행렬 곱셈과 같음을 보여준다.

만약 $$R = R_{x, \theta}$$인 경우 $$S$$를 다음과 같이 계산할 수 있다.

$$
S = \frac{dR}{d\theta}R^T
= \begin{bmatrix} 0 & 0 & 0 \\ 0 & -s_{\theta} & -c_{\theta} \\ 0 & c_{\theta} & -s_{\theta}\end{bmatrix} \begin{bmatrix} 0 & 0 & 0 \\ 0 & c_{\theta} & s_{\theta} \\ 0 & -s_{\theta} & c_{\theta}\end{bmatrix} \\
= \begin{bmatrix} 0 & 0 & 0 \\ 0 & 0 & -1 \\ 0 & 1 & 0\end{bmatrix} = S(i)
$$

따라서, x 축에 대한 $$\theta$$ 만큼의 회전 변환 행렬의 도함수는 다음과 같이 나타낼 수 있다.

$$
\frac{dR_{x,\theta}}{d\theta} = S(i)R_{x,\theta}
$$

이와 같은 형태로, y축, z축 회전 행렬의 도함수는 다음처럼 정리할 수 있다.

$$
\frac{dR_{y,\theta}}{d\theta} = S(j)R_{y,\theta}, \;\frac{dR_{z,\theta}}{d\theta} = S(k)R_{z,\theta}
$$

## 4. Angular Velocity 구하기

회전행렬 $$R$$이 시간에 따라 변한다고 가정하면, $$R = R(t) \in SO(3)$$라고 가정할 수 있다.

그러면 $$R(t)$$를 시간 $$t$$에 대하여 미분하면 다음과 같다.

$$
\dot{R}(t) = S(w(t))R(t)
$$

여기서 $$S(w(t))$$는 skew symmetric 행렬이고, $$w(t)$$는 시간 $$t$$에서 고정된 좌표계에 대한 현재 좌표계의 각속도 값이다.

움직이고 있는 현재 좌표계 위의 한 점 $$p$$를 고려하면, 고정 좌표계에 대한 점 $$p$$의 벡터는 $$p^0 = R^0_1p^1$$으로 나타낼 수 있다.

이를 미분하면 다음과 같다.

$$
\frac{d}{dt}p^0 = \dot{R}^0_1p^1 = S(w)R^0_1p^1 = w \times R^0_1p^1 = w \times p^0
$$

$$p^1$$은 상수이기 때문에 미분값이 0이 되고, $$w$$는 각속도 벡터이다. 따라서 $$p^0$$의 미분값은 $$w \times p^0$$가 되고, 이는 모멘텀(momentum) 값의 형태와 같다.

만약 예를 들어 $$R = R_{x,\theta}$$를 미분해보자.

$$
\dot{R} = \frac{dR}{dt} = \frac{dR}{d\theta}\frac{d\theta}{dt} = \dot{\theta}S(\hat{i})R(t) = S(w(t))R(t)
$$

여기서 $$w = \hat{i}\dot{\theta}이고, 각속도를 뜻한다.

여기서 $$\hat{i} = \begin{bmatrix} 1 & 0 & 0 \end{bmatrix}$$ 이다.

이를 일반화하면 다음과 같다.

$$
\dot{R}_{x,\psi} = S(\dot{\psi}\hat{i})R_{x,\psi} \\
\dot{R}_{y,\theta} = S(\dot{\theta}\hat{j})R_{y,\theta} \\
\dot{R}_{z,\phi} = S(\dot{\phi}\hat{i})R_{x,\phi}
$$

이제 고정 좌표계 $$o_0x_0y_0z_0$$ 위에 있는 두 개의 좌표계 $$o_1x_1y_1z_1$$, $$o_2x_2y_2z_2$$ 간의 각속도 관계를 구해보자.

세 계의 좌표계는 다음과 같은 관계를 가지고 있다고 가정한다.

$$
R^0_2(t) = R^0_1(t)R^1_2(t)
$$

위 식의 양 변을 미분하면 다음과 같다.

$$
\dot{R}^0_2 = \dot{R}^0_1R^1_2 + R^0_1\dot{R}^1_2
$$

위 식의 좌변을 풀면 다음과 같다.

$$
\dot{R}^0_2 = S(w^0_{0,2})R^0_2
$$

여기서 $$w^k_{i,j}$$는 프레임 $$k$$에 대한 $$R^i_j$$를 미분했을 때의 각속도 벡터이다.

그리고 우변을 각각 풀면 다음과 같다.

$$
\dot{R}^0_1R^1_2 = S(w^0_{0,1})R^0_1R^1_2 = S(w^0_{0,1})R^0_2 \\
R^0_1\dot{R}^1_2 = R^0_1S(w^1_{1,2})R^1_2 = R^0_1S(w^1_{1,2})(R^0_1)TR^0_1R^1_2 = S(R^0_1w^1_{1,2})R^0_1R^1_2 = S(R^0_1w^1_{1,2})R^0_2
$$

이에 따라 식을 다시 표현하면 최종적으로 다음과 같은 형태가 된다.

$$
S(w^0_2)R^0_2 = \lbrace {S(w^0_{0,1})} + S(R^0_1w^1_{1,2}) \rbrace R^0_2
$$

skew symmetric의 성질인 $$S(a)+S(b) = S(a+b)$$로 인하여 다음과 같이 정리할 수 있다.

$$
w^0_2 = w^0_{0,1} + R^0_1w^1_{1,2}
$$

이는 $$d^0_2 = d^0_1 + R^0_1d^1_2$$의 형태와 비슷하다.

이를 일반화하여 0번 고정 좌표계에 대한 n번 좌표계의 회전 행렬의 도함수는 다음과 같다.

$$
\dot{R}^0_n = S(w^0_{0,n})R^0_n
$$

여기서 $$w^0_{0,n}$$는 다음과 같다.

$$
w^0_{0,n} = w^0_{0,1} + R^0_1w^1_{1,2} + R^0_2w^2_{2,3} + R^0_3w^3_{3,4} + \dots + R^0_{n-1}w^n-1_{n-1,n} \\
 = w^0_{0,1} + w^0_{1,2} + w^0_{2,3} + w^0_{3,4} + \dots + w^0_{n-1,n}
$$

## 5. Linear Velocity 구하기

시간에 따른 Homogeneous Transformation은 다음과 같다.

$$
H^0_1(t) = \begin{bmatrix} R^0_1(t) & o^0_1(t) \\ 0 & 1 \end{bmatrix}
$$

1번 좌표계에 고정되어 있는 점 $$p$$에 대하여 0번 좌표계에 대한 값으로 변환하는 식은 다음과 같다.

$$
\begin{bmatrix} p^0 \\ 1 \end{bmatrix} = \begin{bmatrix} R^0_1(t) & o^0_1(t) \\ 0 & 1 \end{bmatrix} \begin{bmatrix} p^1 \\ 1 \end{bmatrix}
$$

$$
p^0 = Rp^1 + o
$$

이를 미분하면 다음과 같다.

$$
\dot{p}^0 = \dot{R}p^1 + \dot{o} = S(w)Rp^1 + \dot{o}= w^0 \times r^0 + v^0
$$

$$r=Rp^1$$은 $$o_0x_0y_0z_0$$ 좌표계에서 바라본 $$o_1$$에서 $$p$$로의 벡터이다. 그리고 $$v$$는 $$o_1$$의 선형 속도 값이다.

> 만약 점 p가 $$o_1x_1y_1z_1$$ 좌표계에 대해서도 시간에 따라 변하는 값이라면 $$R(t)\dot{p}^1$$ 항이 추가되어야 한다.

## 6. Jacobian 구하기

n개의 링크와 $$q_1, \dots, q_n$$의 조인트를 가진 매니퓰레이터의 변환행렬은 다음과 같다.

$$
T^0_n(q) = \begin{bmatrix} R^0_n(q) & o^0_n(q) \\ 0 & 1 \end{bmatrix}
$$

여기서 $$q = \begin{bmatrix} q_1, \dots, q_n \end{bmatrix}^T$$는 joint variable에 대한 백터이다.

앞에서 구한 관계에 따라 다음과 같이 정리할 수 있다.

$$
S(w^0_n) = \dot{R}^0_n(R^0_n)^T \\
v^0_n = \dot{o}^0_n
$$

End Effector의 속도는 다음과 같이 표현할 수 있다.

$$
\dot{X} = \begin{bmatrix} v^0_n \\ w^0_n \end{bmatrix} = \begin{bmatrix} J_v \\ J_w \end{bmatrix} \begin{bmatrix} \dot{q}_1 \\ \dots \\ \dot{q}_n \end{bmatrix}
$$

여기서 $$\dot{q}(t) = \begin{bmatrix} \dot{q}_1 \\ \dots \\ \dot{q}_n \end{bmatrix}$$은 각 joint variable의 속도값이고, $$v^0_n$$, $$w^0_n$$는 $$3 \times 1$$행렬, $$J_v$$, $$J_w$$는 $$3 \times n$$행렬이다.

여기서 $$J_v$$, $$J_w$$가 구하고자 하는 Jacobian이 된다. 

### 6.1 Angular Velocity

이제 $$J_w$$를 구하고자 한다. 만약 i 번째 joint가 회전 조인트라면, i-1번 프레임에 대한 angular velcity는 다음과 같이 표현할 수 있다.

$$
w^{i-1}_i = \dot{q}_iz^{i-1|}_{i-1} = \dot{q}_i\hat{k}
$$

단, D-H convention을 만족한 경우에만 위 식이 성립한다.

$$w^{i-1}_i$$는 $$o_{i-1}x_{i-1}y_{i-1}z_{i-1}$$ 좌표계에 붙어있는 joint i의 회전에 의해 생기는 link i의 회전 속도이고, $$\hat{k}$$는 $$\begin{bmatrix} 0 & 0 & 1 \end{bmatrix}^T$$인 단위벡터이다.

만약 i 번째 조인트가 prismatic 조인트라면 $$w^{i-1}_i = 0$$이 된다. 즉, i 번째 조인트가 회전 조인트인 경우에만 End Effector의 각속도에 영향을 준다.

결과적으로 End Effector의 회전 속도 $$w^0_1$$는 다음과 같다.

$$
w^0_n = \rho_1 \dot{q}_1 \hat{k} + \rho_2 \dot{q}_2 R^0_1 \hat{k} + \dots + \rho_n \dot{q}_n R^0_n \hat{k} = \displaystyle\sum_{i=1}^{n}{\rho_i \dot{q}_i z^0_{i-1}}
$$

여기서 $$\rho_i$$는 i 번째 joint가 회전 조인트인 경우 1, prismatic이면 0이고, $$z^0_{i-1} = R^0_{i-1} \hat{k}$$이다.

따라서 $$J_w$$는 다음과 같이 구할 수 있다.

$$
J_w = \begin{bmatrix} \rho_1 z^0_0 & \dots & \rho_n z^0_{n-1} \end{bmatrix}
$$

### 6.2 Linear Velocity

다음으로 $$J_v$$를 구한다. End Effector의 선형 속도는 revloute, prismatic 조인트에 모두 영향을 받는다. 

$$
\dot{o}^0_n = \displaystyle\sum_{i=1}^{n}{\frac{\partial o^0_n}{\partial q_i} \dot{q}_i} = \displaystyle\sum_{i=1}^{n}{J_{v_i} \dot{q}_i}
$$

그러므로 $$J_{v_i} = \frac{\partial o^0_n}{\partial q_i}$$가 된다. 이는 식은 i 관절만 움직이고 나머지 관절은 움직이지 않을 때 End Effector의 움직임을 나타내는데, 0번 joint부터 n번 조인트까지의 합이 최종적으로 End Effector의 선형 속도가 될 것이다.

#### Prismatic Joint

Prismatic Joint에서의 $$J_{v_i}$$ 구할 것이다.

<img src="/assets/img/posts/230429_prismatic_joints.png">

위 그림에서 모든 조인트는 고정되어 있고, joint i만 단독으로 움직이는 prismatic joint라고 가정한다. prismatic joint이기 때문에, 해당 joint가 움직이면 End Effector의 움직임에도 그대로 반영된다. 움직이는 방향은 $$z_{i-1}$$방향이고, 그 속도는 $$\dot{d}_i$$이다. 따라서 다음과 같이 표현할 수 있다.

$$
\dot{o}^0_n = \dot{d}_iR^0_{i-1} \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} = \dot{d}_i z^0_{i-1}
$$

즉, prismatic joint에서의 $$J_{v_i}$$는 다음과 같다.

$$
J_{v_i} = z_{i-1}
$$

#### Revolute Joint

다음으로 Revolute Joint에서의 $$J_{v_i}$$ 구할 것이다.

<img src="/assets/img/posts/230429_revolute_joints.png">

위 그림에서도 마찬가지로 모든 조인트는 고정되어 있고, joint i만 단독으로 움직이는 revolute joint라고 가정한다. revolute joint이기 때문에, 해당 joint가 움직이면 End Effector는 $$w \times r$$의 속도로 움직인다.

여기서 $$w$$와 $$r$$은 다음과 같다.

$$
w = \dot{\theta}_iz_{i-1} \\
r = o_n - o_{i-1}
$$

따라서 revolute joint에서의 $$J_{v_i}$$는 다음과 같다.

$$
J_{v_i} = z_{i-1} \times (o_n - o_{i-1})
$$

### 6.3 Combining Linear and Angular Velocity Jacobian

이제 Linear Velocity Jacobian과 Angular Velocity Jacobian을 합쳐 하나의 Jacobian을 구할 것이다.

Jacobian의 처음 세 개의 행은 $$J_v$$가 되고, 마지막 세 개의 행은 $$J_w$$가 된다.

$$J_v$$는 prismatic joint와 revolute joint의 경우로 나뉘어졌다.

만약 i번째 joint가 revolute joint 라면 $$z_{i-1} \times (o_n - o_{n-1})이고, prismatic joint라면 $$z_{i-1}$$이 된다.

다음으로 $$J_w$$는 i번째 joint가 revolute joint인 경우에만 영향을 받았다. 따라서, i번째 joint가 revolute joint인 경우에 $$z_{i-1}$$이 되고, prismatic joint라면 $$0$$ 벡터가 된다.

모든 joint의 jacobian 값을 합쳐 하나의 Jacobian을 표현하면 다음과 같다.

$$
J = \begin{bmatrix} J_1 & J_2 & \dots & J_n \end{bmatrix}
$$

그리고 각 $$J_i$는,

revolute joint일 때,

$$
J_i = \begin{bmatrix} z_{i-1} \times (o_n - o_{i-1}) \\ z_{i-1} \end{bmatrix}
$$

prismatic joint일 때,

$$
J_i = \begin{bmatrix} z_{i-1} \\ 0 \end{bmatrix}
$$

로 표현할 수 있다.

> Jacobian은 결국 $$z_i$$와 $$o_i$$를 계산하면 구할 수 있다. $$z_i$$는 변환 행렬 $$T^0_i$$에서의 세 번째 컬럼의 처음 세 개의 요소에 해당하고, $$o_i$$는 네 번째 컬럼의 처음 세 개의 요소에 해당한다. 즉, FK를 통해 얻어진 변환 행렬을 통해 Jacobian을 유도할 수 있는 셈이다.

### 6.4 Jacobian 예제

#### 2d planar 

<img src="/assets/img/posts/230429_planar_manipulator.png">

위와 같이 2개의 revolute joint를 가진 2d planar 로봇의 Jacobian matrix(6x2)를 구하면 다음과 같다.

$$
J(q) = \begin{bmatrix} z_0 \times (o_2 - o_0) & z_1 \times (o_2 - o_1) \\ z_0 & z_1 \end{bmatrix}
$$

각 변수는 다음과 같다.

$$
o_0 = \begin{bmatrix} 0 \\ 0 \\ 0 \end{bmatrix}, \;
o_1 = \begin{bmatrix} a_1c_1 \\ a_1s_1 \\ 0 \end{bmatrix}, \;
o_2 = \begin{bmatrix} a_1c_1 + a_2c_{12} \\ a_1s_1 + a_2s_{12} \\0 \end{bmatrix}, \;
z_0 = z_1 = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}
$$

결과적으로 Jacobian은 다음과 같이 구할 수 있다.

$$
J = \begin{bmatrix} -a_1s_1 - a_2s_{12} & -a_2s_{12} \\
a_1c_1 + a_2c_{12} & a_2c_{12} \\
0 & 0 \\
0 & 0 \\
0 & 0 \\
1 & 1 \end{bmatrix}
$$

만약 $$o_2$$ 지점이 아닌, link 2의 가운데 지점인 $$o_c$$ 지점에 대한 Jacobian 관계를 구한다면 다음과 같을 것이다. 단, $$o_c$$에 대한 벡터는 계산되어야 한다.

$$
J(q) = \begin{bmatrix} z_0 \times (o_c - o_0) & z_1 \times (o_c - o_1) \\ z_0 & z_1 \end{bmatrix}
$$

#### Standford manipulator

<img src="/assets/img/posts/230429_standford_arm.png">

Standford 매니퓰레이터의 3번째 joint는 prismatic이고, $$o_3=o_4=o_5=o$$라는 특성을 가진다.

Jacobian 식을 나타내면 다음가 같다.

$$
J = \begin{bmatrix}
z_0 \times (o_6 - o_0) & z_1 \times (o_6 - o_1) & z_2 & z_3 \times (o_6 - o) & z_4 \times (o_6 - o) & z_5 \times (o_6 - o_0) \\
z_0 & z_1 & 0 & z_3 & z_4 & z_5
\end{bmatrix}
$$

여기서 $$o_0 = o_1 = o_2$$, $$o_3$$, $$o_6$$는 $$T^0_j$$ 행렬의 마지막 컬럼을 통해서 구할 수 있고, 각 조인트의 축 벡터는 $$z_j = R^0_jk$$와 같이 구할 수 있다.

$$o_j$$는 다음과 같다.

$$
o_0 = \begin{bmatrix} 0 \\ 0 \\ 0 \end{bmatrix} \\
\\
o_3 = \begin{bmatrix} c_1s_2d_3 - s_1d_2 \\ s_1s_2d_3 + c_1d_2 \\ c_2d_3 \end{bmatrix}
\\
o_6 = \begin{bmatrix}
c_1s_2d_3 - s_1d_2 + d_6(c_1c_2c_4s_5 + c_1c_5s_2 - s_1s_4s_5) \\
s_1s_2d_3 - c_1d_2 + d_6(c_1s_4s_5 + c_2c_4s_1s_5 + c_5s_1s_2) \\
c_2d_3 + d_6(c_2c_5 - c_4s_2s_5)
\end{bmatrix}
$$

$$z_j$$는 다음과 같다.

$$
z_0=\begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix},
z_1=\begin{bmatrix} -s_1 \\ c_1 \\ 0 \end{bmatrix},
z_2=\begin{bmatrix} c_1s_2 \\ s_1s_2 \\ c_2 \end{bmatrix},
z_3=\begin{bmatrix} c_1s_2 \\ s_1s_2 \\ c_2 \end{bmatrix},
z_4=\begin{bmatrix} -c_1c_2s_4-s_1c_4 \\ -s_1c_2s_4+c_1c_4 \\ s_2s_4 \end{bmatrix},
z_5=\begin{bmatrix} c_1c_2c_4s_5-s_1s_4s_5+c_1s_2c_5 \\ s_1c_2c_4s_5+c_1s_4s_5+s_1s_2c_5 \\ -s_2c_4s_5+c_2c_5 \end{bmatrix}
$$

## 7. Analytical Jacobian

지금까지 유도한 Jacobian은 Geometric Jacobian이라고 한다면, 이제부터는 최소한의 표현식으로 나타내저이즌 Analytical Jacobian $$J_a(q)$$를 구할 것이다.

$$
X = \begin{bmatrix} d(q) \\ \alpha(q) \end{bmatrix}
$$

$$X$$는 End Effector의 pose이고, $$d(q)$$는 베이스 프레임에서 End Effector 프레임의 방향 벡터, $$\alpha(q)$$는 베이스 프레임에 대한 End Effector의 방향을 나타낸 표현식이다. 예를 들어, $$\alpha = \begin{bmatrix} \phi & \theta & \psi \end{bmatrix}^T$$와 같은 Eular Angle로 나타낸 표현식이다.

식의 양 변을 미분하면 다음과 같을 것이다.

$$
\dot{X} = \begin{bmatrix} \dot{d} \\ \dot{\alpha} \end{bmatrix} = J_a(q)\dot{q}
$$

여기서 $$J_a(q)$$가 Analytical Jacobian이다.

만약 $$R = R_{z,\psi}R_{y,\theta}R_{z,\phi}$$(Euler angle)이라면,

$$
\dot{R} = \dot{R}_zR_yR_z + R_z\dot{R}_yR_z + R_zR_y\dot{R_z} = S(w)R
$$

와 같고, 위의 관계에서 최종적으로 $$w$$는 다음과 같이 구할 수 있다.

$$
w = \begin{bmatrix}
c_\psi s_\theta \dot{\phi} - s_\psi \dot{\theta} \\
s_\psi s_\theta \dot{\phi} + c_\psi \dot{\theta} \\
\dot{\psi} + c_\theta \dot{\phi}
\end{bmatrix}
= \begin{bmatrix}
c_\psi s_\theta & -s_\psi & 0 \\
s_\psi s_\theta & c_\psi & 0 \\
c_\theta & 0 & 1
\end{bmatrix}
\begin{bmatrix}
\dot{\phi} \\ \dot{\theta} \\ \dot{\psi}
\end{bmatrix}
$$

이 식을 다음과 같이 표현하자.

$$
w = B(\alpha) \dot{\alpha}
$$

이렇게 구한 식을 Jacobian 관계식에 대입하면 다음과 같다.

$$
J(q)\dot{q} = 
\begin{bmatrix} v \\ w \end{bmatrix} =
\begin{bmatrix} \dot{d} \\ B(\alpha)\dot{\alpha} \end{bmatrix} =
\begin{bmatrix} I & 0 \\ 0 & B(\alpha) \end{bmatrix}
\begin{bmatrix} \dot{d} \\ \dot{\alpha} \end{bmatrix} =
\begin{bmatrix} I & 0 \\ 0 & B(\alpha) \end{bmatrix}J_a(q)\dot{q}
$$

따라서 Analytical Jacobian과 Geometric Jacobian의 관계는 다음과 같다.

$$
J_a(q) = \begin{bmatrix} I & 0 \\ 0 & B^{-1}(\alpha) \end{bmatrix}J(q) 
$$

여기서 $$B^{-1}$$은 다음과 같다.

$$
B^{-1}(\alpha) = 
\begin{bmatrix}
\frac{c_\psi}{s_\theta} & \frac{s_\psi}{s_\theta} & 0 \\
-s_\psi & c_\psi & 0 \\
-\frac{c_\psi}{t_\theta} & -\frac{s_\psi}{t_\theta} & 1
\end{bmatrix}
$$

만약, Euler Angle의 변환이 Z($$\psi$$), Y($$\theta$$), X($$\phi$$) 순서라면, 

$$
\begin{bmatrix} w_x \\ w_y \\ w_z \end{bmatrix} = 
B(\alpha) \dot{\alpha} =
\begin{bmatrix} c_\psi c_\theta & -s_\psi & 0 \\
s_\psi c_\theta & c_\psi & 0 \\
-s_\theta & 0 & 1 \end{bmatrix}
\begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix}
$$

이고,

$$
\begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix} = B^{-1} \begin{bmatrix} w_x \\ w_y \\ w_z \end{bmatrix} =
\begin{bmatrix}
\frac{c_\psi}{c_\theta} & \frac{s_\psi}{c_\theta} & 0 \\
-s_\psi & c_\psi & 0 \\
c_\psi t_\theta & s_\psi t_\theta & 1
\end{bmatrix}
\begin{bmatrix} w_x \\ w_y \\ w_z \end{bmatrix}
$$

의 관계를 가진다. 이는 End Effector에 있는 Gyro Sensor의 $$\begin{bmatrix} w_x \\ w_y \\ w_z \end{bmatrix}$$ 값을 이용하여, 베이스 프레임에 대한 End Effector의 Euler Angle 각속도를 구할 수 있음을 보여준다.
