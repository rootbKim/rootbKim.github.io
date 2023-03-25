---
layout: post
title: 로봇동역학 - Rigid Motions and Homogeneous Transformations
feature-img: "assets/img/posts/dynamics.webp"
thumbnail: "assets/img/posts/dynamics.webp"
tags: [Robot Dynamics]
---

로봇동역학에서 기본이 되는 회전 행렬 및 Homogeneous 변환 행렬에 대해서 정리한다.

## 1. Orientation Matrix(Rotation Matrix)

2차원 평면 상에서 $$o_0x_0y_0$$ 좌표계에 대하여 $$\theta$$만큼 회전한 $$o_1x_1y_1$$ 좌표계의 Rotation Matrix $$R^0_1$$는 다음과 같다.

$$
R^0_1 = [x^o_1 | y^0_1] = \begin{bmatrix}cos\theta & -sin\theta \\ sin\theta & cos\theta \\ \end{bmatrix} = \begin{bmatrix}x_1 \cdot x_0 & y_1 \cdot x_0 \\ x_1 \cdot y_0 & y_1 \cdot y_0 \\ \end{bmatrix}
$$

3차원 공간에서 $$o_0x_0y_0z_0$$ 좌표계에 대하여 $$\theta$$만큼 회전한 $$o_1x_1y_1z_1$$ 좌표계의 Rotation Matrix $$R^0_1$$는 다음과 같다.

$$
R^0_1 = \begin{bmatrix}x_1 \cdot x_0 & y_1 \cdot x_0 & z_1 \cdot x_0 \\ x_1 \cdot y_0 & y_1 \cdot y_0 & z_1 \cdot y_0\\ x_1 \cdot z_0 & y_1 \cdot z_0 & z_1 \cdot z_0 \end{bmatrix}
$$

여기서 만약 z축 회전을 하면

$$
x_1 \cdot x_0 = cos\theta \\
y_1 \cdot x_0 = -sin\theta \\
x_1 \cdot y_0 = sin\theta \\
y_1 \cdot y_0 = cos\theta \\
z_0 \cdot z_1 = 1 \\
$$

이고, 그 이외의 내적 값은 0이 되므로 회전 행렬은 다음과 같다.

$$
R^0_1 = \begin{bmatrix} cos\theta & -sin\theta & 0 \\ sin\theta & cos\theta & 0 \\ 0 & 0 & 1 \end{bmatrix}
$$

이와 같이 각각의 축을 기준으로 회전하는 경우의 회전 행렬은 다음과 같이 정리할 수 있다.

$$
R_{z,\theta} = \begin{bmatrix} cos\theta & -sin\theta & 0 \\ sin\theta & cos\theta & 0 \\ 0 & 0 & 1 \end{bmatrix} \\
R_{x,\theta} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & cos\theta & -sin\theta  \\ 0 & sin\theta & cos\theta \end{bmatrix} \\
R_{y,\theta} = \begin{bmatrix} cos\theta & 0 & sin\theta \\ 0 & 1 & 0 \\ -sin\theta & 0 & cos\theta \end{bmatrix}
$$

## 2. 현재 좌표계에 대한 회전

현재 좌표계에 대한 회전은 각 프레임이 이전 프레임에 대하여 회전하는 경우를 말한다.

0번 Frame에서 본 1번 Frame의 회전 행렬이 $$R^0_1$$일 때, 0번 Frame에서의 $$p$$ 벡터 $$p^0$$와 1번 Frame에서의 $$p$$ 벡터 $$p^1$$의 관계는 다음과 같다.

$$
p^0 = R^0_1p^1
$$

위의 상황에서 2번 좌표계가 1번 좌표계에 대하여 회전된 좌표계라면, 2번 좌표계와 0번 좌표계 관의 관계 $$R^0_2$$는 다음과 같이 정의된다.

$$
p^1 = R^1_2p^2\\
p^0 = R^0_1R1_2p^2 = R^0_2p^2
$$

따라서

$$
R^0_2 = R^0_1R^1_2
$$

가 된다. 즉, $$R^0_2$$는 0번 좌표계에 대하여 회전된 2번 좌표계와의 회전행렬이다.

> 좌표변환 순서에 따라 회전 행렬도 달라지므로, 순서에 주의해야한다.

## 3. 고정 좌표계에 대한 회전

고정 좌표계에 대한 회전은 고정된 글로벌한 좌표계에 대하여 항상 회전을 고려하는 경우로, 현재 화표계에 대한 회전과 회전 행렬의 순서가 반대가 된다.

글로벌한 좌표계인 0번 좌표계를 기준으로 1번 좌표계를 회전시키고, 1번 좌표계 위에 있는 2번 좌표계는 1번 좌표계의 회전을 고려하지 않고, 0번 좌표계 기준으로 회전을 시키는 경우이다.

$$
R^0_2 = R^0_1R^1_2 = R^0_1[(R^0_1)^{-1}RR^0_1] = RR^0_1
$$

여기서 $$R^1_2$$는 이전 프레임의 회전을 고려하지 않기 때문에 $$R^0_1$$을 곱하고, 0번 좌표계에 대한 2번 좌표계의 회전인 $$R$$을 곱하고, 마지막으로 다시 이전 프레임의 회전을 고려하기 위하여 $$R^0_1$$을 곱한 결과 $$R^0_2$$가 $$RR^0_1$$이 된다.

이 순서는 회전 좌표계에서의 회전과 반대되는 순서이다.

## 4. Euler Angles

현재 좌표계를 사용하며, $$(\phi, \theta, \psi)$$ 각도로 세 번 회전한 좌표계로, 그 순서는 다양하나, 대표적으로 z, y, z 순서로 회전한 좌표계가 대표적이다. 즉, 오일러 회전 행렬은 다음과 같다.

$$
R^0_1 = R_{z,\phi}R_{y,\theta},R_{z,\psi} \\
 = \begin{bmatrix} c_\phi & -s_\phi & 0 \\ s_\phi & c_\phi & 0 \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} c_\theta & 0 & s_\theta \\ 0 & 1 & 0 \\ -s_\theta & 0 & c_\theta \end{bmatrix} \begin{bmatrix} c_\psi & -s_\psi & 0 \\ s_\psi & c_\psi & 0 \\ 0 & 0 & 1 \end{bmatrix} \\
 = \begin{bmatrix} c_\phi c_\theta c_\psi - s_\phi s_\psi & -c_\phi c_\theta s_\psi - s_\phi c_\psi & c_\phi s_\theta \\ s_\phi c_\theta c_\psi + c_\phi s_\psi & -s_\phi c_\theta s_\psi + c_\phi c_\psi & s_\phi s_\theta \\ -s_\theta c_\psi & s_\theta s_\psi & c_\theta \end{bmatrix}
$$

이 $$R^0_1$$를 나타내기 쉽게 다음과 같이 표현하자.

$$
R^0_1 = \begin{bmatrix} r_{11} & r_{12} & r_{13} \\ r_{21} & r_{22} & r_{23} \\ r_{31} & r_{32} & r_{33} \end{bmatrix}
$$

해는 $$r_{31} \neq 0, r_{32} \neq 0, r_{33} \neq \pm 1$$(또는, $$r_{13} \neq 0, r_{23} \neq 0, s_\theta \neq 0$$ 의 조건이 성립하는 경우 $$\theta$$는 다음과 같이 값을 구할 수 있다.

$$
\theta = Atan2(\pm \sqrt{1-{r_{33}}^2}, r_{33})
$$

$$\theta$$ 값에 따라서 다음과 같이 두 가지의 경우로 해를 구할 수 있다.

$$
if \;\; \theta > 0 \\
\phi = Atan2(r_{23}, r_{13}) \\
\psi = Atan2(r_{32}, -r_{31}) \\
$$

또는

$$
if \;\; \theta < 0
\phi = Atan2(-r_{23}, -r_{13}) \\
\psi = Atan2(-r_{32}, r_{31})
$$


만약에 $$r_{13} = r_{23} = 0, r_{33} = \pm 1 (\theta = 0)$$ 와 같다면, 무수히 많은 해가 존재하게 된다. 이와 같이 z, y, z 순서의 Euler Angle은 $$sin\theta = 0$$ 이면 정의가 불가능하다.

## 5. Roll, Pitch Yaw Angles

고정 좌표계를 사용한 방식으로 고정 좌표계 기준으로 x, y, z 회전한 행렬이다. 이는 Euler Angle의 z, y, x 회전과 동일하다. 여기서 Roll은 $$\phi$$, Pitch는 $$\theta$$, Yaw는 $$\psi$$가 된다.

$$
R^0_1 = R_{z, \phi}R_{y, \theta}R_{x, \psi}
 = \begin{bmatrix} c_\phi c_\theta & -s_\phi c_\psi + c_\phi s_\theta s_\psi & s_\phi s_\psi + c_\phi s_\theta c_\psi \\ s_\phi c_\theta & c_\phi c_\psi + s_\phi s_\theta s_\psi & -c_\phi s_\psi + s_\phi s_\theta c_\psi \\ -s_\theta & c_\theta s_\psi & c_\theta c_\psi \end{bmatrix}
$$

이 회전행렬에서의 해는 다음과 같다.

$$
\phi = Atan2(r_{21}, r{11}) \quad (if \, -\pi / 2 < \theta < \pi / 2) \\
or \\
\phi = Atan2(-r_{21}, -r{11}) \quad (if \, -\pi / 2 > \theta \; or \; \theta > \pi / 2) \\
\theta = Atan2(-r_{31}, c_\phi r_{11} + s_\phi r_{21})
\psi = Atan2(s_\phi r_{31} - c_\phi r_{23}, -s_\phi r_{12} + c_\phi r_{22})
$$

## 6. Homogeneous Transformations

0번 좌표계에 대하여 회전한 1번 좌표계를 $$d^0_1$$만큼 선형변환하면 $$p^0$$는 다음과 같다.

$$
p^0 = R^0_1p^1 +d^0_1
$$

1번 좌표계에 대한 2번 좌표계도 고려하면

$$
p^1 = R^1_2p^2 +d^1_2
$$

가 되고, 위의 두 식은 다음과 같이 나타낼 수 있다.

$$
p^0 = R^0_1R^1_2p^2 + R^0_1d^1_2 + d^0_1
$$

1번 좌표계를 고려하지 않고, 0번 좌표계애 대한 2번 좌표계의 관계도 다음과 같이 나타낼 수 있는데,

$$
p^0 = R^0_2p^2 + d^0_2
$$

이는 

$$
R^0_2 = R^0_1R^1_2
$$

이므로,

$$
d^0_2 = d^0_1 + R^0_1d^1_2
$$

임을 알 수 있다.

이를 행렬로 나타내면 아래와 같이 나타낼 수 있다.

$$
\begin{bmatrix} R^0_1 & d^0_1 \\ 0 & 1 \end{bmatrix}
\begin{bmatrix} R^1_2 & d^1_2 \\ 0 & 1 \end{bmatrix}
= \begin{bmatrix} R^0_1R^1_2 & R^0_1d^1_2 + d^0_1 \\ 0 & 1 \end{bmatrix}
$$

이 행렬이 바로 Homogeneous Transformation이다.

$$
H = \begin{bmatrix} R & d \\ \bold{0} & 1 \end{bmatrix}
$$

여기서 $$\bold{0}$$은 $$(0, 0, 0)$$ row 벡터이다.

> $$ H^{-1} = \begin{bmatrix} R^T & -R^Td \\ \bold{0} & 1 \end{bmatrix}$$

Homogeneous Transformation을 이용하여 $$p^0$$와 $$p^1$$의 관계를 나타내면 다음과 같아지게 된다.

$$
p^0 = H^0_1p^1
$$

이 Homogeneous transformation matrix는 x,y,z 각각의 좌표축에 대한 선형 변환과 회전 변환에 대하여 다음과 같이 정리된다.

$$
Trans_{x,a} = \begin{bmatrix} 1 & 0 & 0 & a \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix} \\
Trans_{y,b} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & b \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix} \\
Trans_{z,c} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & c \\ 0 & 0 & 0 & 1 \end{bmatrix} \\
Rot_{x,\alpha} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & c_{\alpha} & -s_{\alpha} & 0 \\ 0 & s_{\alpha} & c_{\alpha} & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix} \\
Rot_{y,\beta} = \begin{bmatrix} c_{\beta} & 0 & s_{\beta} & 0 \\ 0 & 1 & 0 & 0 \\ -s_{\beta} & 0 & c_{\beta} & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix} \\
Rot_{z,\gamma} = \begin{bmatrix} c_{\gamma} & -s_{\gamma} & 0 & 0 \\ s_{\gamma} & c_{\gamma} & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix} \\
$$

이 6가지 Homogeneous transformations은 3차원 공간에서의 기본 변환 행렬이 된다.

예를 들어, x축으로 $$a$$ 만큼 이동하고, y축을 중심으로 $$\beta$$ 만큼 회전하면 이 때의 변환 행렬은 다음과 같이 된다.

$$
R = Trans_{x,a}Rot_{y,\beta} = \begin{bmatrix} 1 & 0 & 0 & a \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}\begin{bmatrix} c_{\beta} & 0 & s_{\beta} & 0 \\ 0 & 1 & 0 & 0 \\ -s_{\beta} & 0 & c_{\beta} & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix} = \begin{bmatrix} c_{\beta} & 0 & s_{\beta} & a \\ 0 & 1 & 0 & 0 \\ -s_{\beta} & 0 & c_{\beta} & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$