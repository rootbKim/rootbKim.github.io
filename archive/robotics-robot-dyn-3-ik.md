---
layout: note_page
title: 로봇동역학 - Inverse Kinematics
tags: [Robot Dynamics]
category: "Robotics"
---

3차원 공간에서의 End Effector의 좌표와 방향이 주어졌을 때 로봇의 각 조인트 변수를 구하는 방법인 Inverse Kinematics(IK)에 대해 정리한다.

## 1. Inverse Kinematics

Forward Kinematics가 로봇의 각 조인트 변수가 주어졌을 때의 End Effector의 좌표를 구하는 방법이었다면, Inverse Kinematics는 3차원 공간에서의 End Effector의 좌표와 방향이 주어졌을 때 로봇의 각 조인트 변수를 구하는 방법이다.

즉, IK는 글로벌 좌표계에 대한 End Effector의 Homogeneous Transformation Matrix $$H$$가 주어진 상황에서 시작한다.

$$
H = \begin{bmatrix} r_{11} & r_{12} & r_{13} & d_x \\ r_{21} & r_{22} & r_{23} & d_y \\ r_{31} & r_{32} & r_{33} & d_z \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

FK에서는 각 조인트의 Homogeneous Transformation Matrix $$A_q$$를 이용하여 $$H$$를 구했으므로, 반대로 풀면 각 조인트 변수를 구할 수 있을 것이다!

$$
A_{q_1}A_{q_2} \dots A_{q_n} = H
$$

하지만 조인트가 많아질 수록 단순히 위 관계식만을 이용하여 각 조인트 변수를 계산하는 것은 쉽지 않다. 이러한 문제를 해결하기 위해 문제를 `Decouple` 단계와 `Geometric` 단계로 나누어 문제를 좀더 간단하게 만들어 솔루션을 계산하는 방법을 사용한다.

다음 그림과 같은 6자유도를 가진 standford manipulator를 생각해보면, 마지막 3개의 조인트는 한 점($$o_c$$)에서 교차하는데, 이는 IK 문제를 **위치(Position)에 대한 IK**와 **방향(Orientation)에 대한 IK 문제**로 **분할(Decouple)**시켜, 문제를 간단하게 만들어 준다.

<img src="/assets/img/posts/230403_standford_manipulator.png" width=500>

## 2. Decouple

$$
R^0_6(q_1 \dots q_6) = R \\
o^0_6(q_1 \dots q_6) = o
$$

$$z_3$$, $$z_4$$, $$z_5$$ 축이 모두 $$o_c$$에서 교차하고, $$o_3$$, $$o_4$$, $$o_5$$가 $$o_c$$와 같은데, 이는 이 세 개의 조인트 변수가 변하더라도 $$o_c$$의 위치는 변하지 않는다. 즉, $$o_c$$의 위치는 $$q_1$$, $$q_2$$, $$q_3$$에 의해서만 변한다.

이러한 특성으로 인해 End Effector의 위치 $$o$$는 다음과 같이 단순화할 수 있다.

$$
o = o^0_c + R^0_5o^5_6 \\

o = o^0_c + R^0_5 \begin{bmatrix} 0 \\ 0 \\ d_6 \end{bmatrix} \\

o^0_c = o - R \begin{bmatrix} 0 \\ 0 \\ d_6 \end{bmatrix}
$$

여기서 $$R$$이 $$R^0_5$$대신 사용할 수 있는 이유는 $$R^0_5$$와 $$R$$이 z축 회전 관계를 갖고 있어, 3번째 열이 같기 때문이다. 이는

$$
R^0_5 \begin{bmatrix} 0 \\ 0 \\ d_6 \end{bmatrix} = R \begin{bmatrix} 0 \\ 0 \\ d_6 \end{bmatrix}
$$

를 성립시킨다.

따라서 다음의 식을 통해 $$o_c$$의 위치를 계산할 수 있다.

$$
\begin{bmatrix} x_c \\ y_c \\ z_c \end{bmatrix} = \begin{bmatrix} o_x - d_6 r_{13} \\ o_y - d_6 r_{23} \\ o_z - d_6 r_{33} \end{bmatrix} \;\;\; where \;\; R = \begin{bmatrix} r_{11} & r_{12} & r_{13} \\ r_{21} & r_{22} & r_{23} \\ r_{31} & r_{32} & r_{33} \end{bmatrix}
$$

다음으로 $$o_3x_3y_3z_3$$ 좌표계에 대한 End Effector의 rotation matrix $$R^3_6$$는 다음과 같이 계산할 수 있다.

$$
R = R^0_6 = R^0_3R^3_6 \\
R^3_6 = (R^0_3)^{-1}R = (R^0_3)^T R
$$

이 $$R^3_6$$를 통해서 $$q_4$$, $$q_5$$, $$q_6$$를 구할 수 있는데, $$R^3_6$$를 구하기 위해서는 $$R^0_3$$를 알아야 한다. 이 문제는 Geometric 방법을 이용한다.

## 3. Geometric

위에서 decouple을 통해서 $$o_c$$의 위치를 구했기 때문에, 이를 Geometric한 방법으로 $$R^0_3$$를 구할 수 있다.

이 문제를 쉽게 해결하기 위해 $$q_i$$ 조인트 값을 구하기 위하여 매니퓰레이터를 $$x_{i-1} - y_{i-1}$$ 좌표평면으로 투사시켜 2차원 공간에서 계산한다.

예를 들어 다음 그림과 같은 elbow manipulator를 $$x_0-y_0$$ 평면에 투사시켜 $$\theta_1$$을 구한다.

<img src="/assets/img/posts/230403_elbow_manipulator.png">

$$\theta_1$$을 구하면 다음과 같다.

$$
\theta_1 = Atan2(y_c, x_c)
$$

또한 $$\theta_1$$은 두 번째 해를 가질 수 있는데, 이 해는 $$\theta_2$$, $$\theta_3$$의 값이 다른 해를 가지게 된다. 하지만 이 두 번째 해는 거의 사용되지 않는다.

$$
\theta_1 = \pi + Atan2(y_c, x_c)
$$

다음으로 $$\theta_2$$, $$\theta_3$$를 구하기 위해 $$x_1-y_1$$ 평면에 투영 시키면 다음과 같이 된다.

<img src="/assets/img/posts/230403_elbow_manipulator_projection.jpg">

해를 구하면 다음과 같다.

$$
\theta_3 = Atan2(\pm \sqrt(1-D^2, D)) \;\;\; where \;\; D = cos \theta_3 = \frac{r^2 + s^2 - a^2_2 - a^2_3}{2a_2a_3} = \frac{(x^2_c + y^2_c - d^2) + (z_c - d_1)^2 - a^2_2 - a^2_3}{2a_2a_3}
$$

$$
\theta_2 = Atan2(s,r) - Atan2(a_3s_3, a_2 + a_3c_3) = Atan2(z_c - d_1, \sqrt(x^2_c + y^2_c - d^2)) - Atan2(a_3s_3, a_2 + a_3c_3)
$$

여기서 $$\theta_3$$도 $$\theta_1$$과 마찬가지로 두 개의 해를 가질 수 있는데, $$\theta_3$$에 따라 elbow-up 형태와, elbow-down 형태가 될 수 있다. 즉, IK를 계산하면 4 개의 해를 구할 수 있게 된다.

## 4. Orientation

앞에서 Geometric 방법으로 $$\theta_1$$, $$\theta_2$$, $$\theta_3$$를 구했기 때문에, 나머지 $$\theta_4$$, $$\theta_5$$, $$\theta_6$$을 구하면 된다. spherical wrist는 `Euler transformation`가 동일한 형태의 rotation matrix를 가진다.

$$
T^3_6 = A_4A_5A_6 = \begin{bmatrix} R^3_6 & o^3_6 \\ 0 & 1 \end{bmatrix} = \begin{bmatrix} c_4 c_5 c_6 - s_4 s_6 & -c_4 c_5 s_6 - s_4 c_6 & c_4 s_5 & c_4 s_5 d_6 \\ s_4 c_5 c_6 + c_4 s_6 & -s_4 c_5 s_6 + c_4 c_6 & s_4 s_5 & s_4 s_5 d_6 \\ -s_5 c_6 & s_5 s_6 & c_5 & c_5 d_6 \\ 0 & 0 & 0 & 1\end{bmatrix}
$$

이 회전변환 행렬의 첫 3x3 부분은 다음의 Euler Transformation 형태와 동일하다.

$$
R^0_1 = R_{z,\phi}R_{y,\theta},R_{z,\psi} \\
 = \begin{bmatrix} c_\phi & -s_\phi & 0 \\ s_\phi & c_\phi & 0 \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} c_\theta & 0 & s_\theta \\ 0 & 1 & 0 \\ -s_\theta & 0 & c_\theta \end{bmatrix} \begin{bmatrix} c_\psi & -s_\psi & 0 \\ s_\psi & c_\psi & 0 \\ 0 & 0 & 1 \end{bmatrix} \\
 = \begin{bmatrix} c_\phi c_\theta c_\psi - s_\phi s_\psi & -c_\phi c_\theta s_\psi - s_\phi c_\psi & c_\phi s_\theta \\ s_\phi c_\theta c_\psi + c_\phi s_\psi & -s_\phi c_\theta s_\psi + c_\phi c_\psi & s_\phi s_\theta \\ -s_\theta c_\psi & s_\theta s_\psi & c_\theta \end{bmatrix}
$$

따라서 다음과 같이 가정할 수 있다.

$$
\theta_4 = \phi \\
\theta_5 = \theta \\
\theta_6 = \psi
$$

이제, $$R^3_6$$을 구하기 위해 위에서 구한 관계를 이용한다.

$$
R^3_6 = (R^0_3)^T R
$$

elbow manipulator의 R^0_3을 구하기 위해 DH 파라미터를 이용하면 다음과 같다.

|Link|$$a_i$$|$$\alpha_i$$|$$d_i$$|$$\theta_i$$|
|-|-|-|-|-|
|1|0|90|$$d_1$$|$$\theta^*_1$$|
|2|$$a_2$$|0|0|$$\theta^*_2$$|
|3|$$a_3$$|0|0|$$\theta^*_3$$|

$$
R^0_3 = \begin{bmatrix} c_1c_{23} & -c_1s_{23} & s_1 \\ s_1c_{23} & -s_1s_{23} & -c_1 \\ s_{23} & c_{23} & 0 \end{bmatrix}
$$

따라서 $$R$$과 $$R^0_3$$을 알기 때문에 $$R^3_6$$을 구할 수 있다.

$$
\begin{bmatrix} c_4 c_5 c_6 - s_4 s_6 & -c_4 c_5 s_6 - s_4 c_6 & c_4 s_5 \\ s_4 c_5 c_6 + c_4 s_6 & -s_4 c_5 s_6 + c_4 c_6 & s_4 s_5 \\ -s_5 c_6 & s_5 s_6 & c_5 \end{bmatrix} = \begin{bmatrix} c_1c_{23} & s_1c_{23} & s_{23} \\ -c_1s_{23} & -s_1s_{23} & c_{23} \\ s_1 & -c_1 & 0 \end{bmatrix} \begin{bmatrix} r_{11} & r_{12} & r_{13} \\ r_{21} & r_{22} & r_{23} \\ r_{31} & r_{32} & r_{33} \end{bmatrix}
$$

이를 풀면 다음과 같은 결과를 얻을 수 있다.

$$
\theta_5 = Atan2(\pm \sqrt(1-(s_1r_{13} - c_1r_{23})^2), sr_{13} - c_1 r_{23})
$$

여기서 $$\theta_5$$의 $$Atan2$$의 첫 번째 인자의 부호에 따라 두 가지 해를 구할 수 있다.

만약 양수 인자를 선택하면 다음과 같은 해를 구할 수 있다.

$$
\theta_4 = Atan2(-c_1s_{23}r_{13} -s_1s_{23}r_{23} + c_{23}r_{33}, c_1c_{23}r_{13} + s_1c_{23}r_{23} + s_{23}r_{33}) \\
\theta_6 = Atan2(s_1r_{12}-c_1r_{22}, -s_1r_{11} + c_1r_{21})
$$

만약 음수 인자를 선택하면 다음과 같은 해를 구할 수 있다.

$$
\theta_4 = Atan2(c_1s_{23}r_{13} + s_1s_{23}r_{23} - c_{23}r_{33}, - c_1c_{23}r_{13} - s_1c_{23}r_{23} - s_{23}r_{33}) \\
\theta_6 = Atan2(- s_1r_{12}+c_1r_{22}, s_1r_{11} - c_1r_{21})
$$

이로써 6개의 조인트 값을 구할 수 있게 되었다.

## 5. Numerical Approach

Numerical하게 IK를 풀 수 있는데, Iteration을 돌며, FK를 이용하여 조인트 변수를 End Effector의 위치 및 방향 오차가 줄어드는 방향으로 업데이트하는 방법이다.

<img src="/assets/img/posts/230403_ik_numerical.png">

이 때 Jacobian을 이용하는데, Analytical Jacobian은 다음과 같다.

$$
\partial x = J \partial \theta \\
J = \begin{bmatrix}
  \frac{\partial x}{\partial \theta_1} & \frac{\partial x}{\partial \theta_2} & \frac{\partial x}{\partial \theta_3} & \frac{\partial x}{\partial \theta_4} & \frac{\partial x}{\partial \theta_5} & \frac{\partial x}{\partial \theta_6} \\

  \frac{\partial y}{\partial \theta_1} & \frac{\partial y}{\partial \theta_2} & \frac{\partial y}{\partial \theta_3} & \frac{\partial y}{\partial \theta_4} & \frac{\partial y}{\partial \theta_5} & \frac{\partial y}{\partial \theta_6} \\
  \
  \frac{\partial z}{\partial \theta_1} & \frac{\partial z}{\partial \theta_2} & \frac{\partial z}{\partial \theta_3} & \frac{\partial z}{\partial \theta_4} & \frac{\partial z}{\partial \theta_5} & \frac{\partial z}{\partial \theta_6} \\

  \frac{\partial \psi}{\partial \theta_1} & \frac{\partial \psi}{\partial \theta_2} & \frac{\partial \psi}{\partial \theta_3} & \frac{\partial \psi}{\partial \theta_4} & \frac{\partial \psi}{\partial \theta_5} & \frac{\partial \psi}{\partial \theta_6} \\

  \frac{\partial \theta}{\partial \theta_1} & \frac{\partial \theta}{\partial \theta_2} & \frac{\partial \theta}{\partial \theta_3} & \frac{\partial \theta}{\partial \theta_4} & \frac{\partial \theta}{\partial \theta_5} & \frac{\partial \theta}{\partial \theta_6} \\

  \frac{\partial \phi}{\partial \theta_1} & \frac{\partial \phi}{\partial \theta_2} & \frac{\partial \phi}{\partial \theta_3} & \frac{\partial \phi}{\partial \theta_4} & \frac{\partial \phi}{\partial \theta_5} & \frac{\partial \phi}{\partial \theta_6} \\
  \end{bmatrix}
$$

이 Jacobian 또한 다음과 같이 Numerical한 방법으로 구할 수 있다.

<img src="/assets/img/posts/230403_jacobian_numerical.png">
