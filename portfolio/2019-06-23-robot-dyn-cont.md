---
layout: portfolio
title: 로봇 동역학 및 제어 텀프로젝트 - 5절링크 로봇의 동역학 해석
feature-img: "/assets/img/portfolio/dyn/dyn.png"
img: "/assets/img/portfolio/dyn/dyn.png"
date: 23 June 2019
excerpt: 로봇 동역학 및 제어 수업의 텀프로젝트로 수행한 5절링크 로봇의 동역학 해석과정을 다룬다.
---

# 개요

로봇 동역학 및 제어 수업의 텀프로젝트로 수행한 5절링크 로봇의 동역학 해석을 진행하였다. 진행 내용으로 FK, IK 및 경로계획을 수행하고, 동역학을 해석하여 관절 제어를 수행한다.

# 관련 링크

* [보고서](/assets/docs/portfolio/dyn.pdf)

# 연구 기간

2019-03-02 ~ 2019-06-23

# 참여 인원

1명

# 기술 스택

- Robotics
- MATLAB
- Webot
- Methematica

# 세부 내용

자세한 수행 내용은 보고서를 참조하며, 여기서는 최종 결과인 독립관절 위치제어와, 다관절 위치제어를 수행한 결과를 다룬다.

### 독립관절 위치제어

* 각 관절에 계획된 경로를 추종할 수 있도록 PD 위치 제어를 수행한 결과이며, 계획된 경로를 잘 추종하는 것을 볼 수 있다.

<img src="/assets/img/portfolio/dyn/pd.gif">

* 그 결과 외부의 힘이 작용할 때에도 강건성을 보이는 것을 알 수 있다.

<img src="/assets/img/portfolio/dyn/pd_force.gif">

### 다관절 위치제어

* 각 관절이 계획된 경로를 추종할 수 있도록 CTC 제어를 수행한 결과이며, 계획된 경로를 잘 추종하는 것을 볼 수 있다.

<img src="/assets/img/portfolio/dyn/ctc.gif">

* CTC 제어는 PD 제어와 다르게 외부의 힘에 유연함을 볼 수 있다.

<img src="/assets/img/portfolio/dyn/ctc_force.gif">

### 각 제어 방법의 비교

* 기존의 경로계획과 토크와 각각 PD, CTC 제어를 수행했을 때의 경로를 비교했을 때, 유사하게 추종함을 볼 수 있었다. 각각 제어의 방법에 따라 게인의 차이에 따라 성능의 차이가 있겠지만, 두 제어 방법 모두 경로 추종을 잘 한다는 것을 알 수 있었다.

<img src="/assets/img/portfolio/dyn/trajectory.png">

* 기존의 로봇의 역동역학 해석 시 나온 크랭크 축의 토크 값과 시뮬레이션에서의 PD, CTC 제어를 수행했을 때의 크랭크 축의 토크 값이다. 이를 볼 때 CTC 제어가 PD 제어보다 조금 더 정확한 토크가 계산되는 것을 알 수 있었다.

<img src="/assets/img/portfolio/dyn/torque.png">

### 다관절 위치제어에서의 경로 추종

* 링크의 끝에 중력 방향으로 사람의 하중이 가해진다고 가정했을 때(보행재활로봇에 사람이 탄다고 가정했을 때), CTC 제어 시 경로 추종 성능과 그에 따른 크랭크 축에 가해지는 토크 값을 데이터화하였다.

<img src="/assets/img/portfolio/dyn/ctc_test.gif">

* 하중이 가해지더라도 기존의 계획된 경로 추종이 잘 되는 것을 알 수 있고, 그에 따른 토크 값도 다음과 같은 결과가 나왔다.

<img src="/assets/img/portfolio/dyn/ctc_trajectory.png">

<img src="/assets/img/portfolio/dyn/ctc_torque.png">

### Conclusion

* Serial Robot 이 아닌 Parallel Robot에서의 동역학 해석에 대해서 공부할 수 있었다.
* 추후 연구 시에는 경로계획에서 로봇의 움직임이 사람의 보행 Trajectory와 유사하도록 Trajectory 를 만들어야 한다.
* 동역학 해석 시 링크를 단순화하여 질량 관성 모멘트와 질량 중심을 구하였는데, 실제 링크의 질량 중심과 관성 모멘트를 고려해야 한다.
* 직접 계산한 Torque와 독립관절제어와 다관절제어에서의 Torque 결과가 상이하게 나왔는데, 크랭크 축이 계속 회전을 하면 이런 문제가 없을 것으로 판단된다. 따라서 Upper arm과 Under arm을 계속해서 운동하는 것을 시뮬레이션 하여 Torque값을 구해야 할 것이다.
* End Effector에 하중이 걸렸을 때 필요한 모터 토크에 대한 해석을 더 진행하여, 모터에 필요한 Torque가 어떻게 나오는지 정확한 해석이 필요하다.