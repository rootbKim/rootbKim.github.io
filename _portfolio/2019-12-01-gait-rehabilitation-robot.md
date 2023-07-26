---
layout: post
title: 역동역학 기반의 파워어시스트 보행재활로봇 훈련 알고리즘 연구
feature-img: "assets/img/portfolio/gait/gait.jpg"
img: "assets/img/portfolio/gait/gait.jpg"
date: 01 December 2019
excerpt: 석사 논문 연구 주제인 '역동역학 기반의 파워어시스트 보행재활로봇 훈련 알고리즘'의 내용을 간단하게 정리한다.
---

## 개요

이 연구는 석사 연구 주제로 보행재활로봇의 형태 중 하나인 말단장치 형태의 로봇(다른 타입으로는 외골격 로봇이 있음.)의 Inverse Dynamics를 이용한 힘 기반 제어 방법에 대한 연구이다. 이 연구를 위하여, 말단장치 형태의 보행재활로봇인 DDgo Pro의 설계와 사용자의 보행을 만들기 위한 로봇의 5절 링크의 Inverse Dynamics 해석, 그리고 이를 기반으로 보행재활 훈련 모드인 Passive Mode, Active Assisted Mode, Active Mode를 구현하였으며, 최종적으로 구현된 보행재활로봇의 재활 효과를 검증하였다.

## 관련 링크

* 논문
    - [Ji-Yong Kim, Jung-Yup Kim, "Gait training algorithm based on inverse dynamics of walking rehabilitation robot, DDgo Pro," Intelligent Service Robotics, Vol. 14, pp. 143-155, 2021.](https://doi.org/10.1007/s11370-021-00357-8)
    - [Jung‑Yup Kim, Ji‑Yong Kim, Hyeong‑Sic Kim, Kiwon Park, "Development and Evaluation of a Hybrid Walking Rehabilitation Robot, DDgo Pro," International Journal of Precision Engineering and Manufacturing, Vol. 21, pp. 2105–2115, 2020.](https://doi.org/10.1007/s12541-020-00404-x)
    - [김지용, 김정엽, "역동역학 기반의 보행 재활 로봇의 수동 모드 보행 훈련 알고리즘", 한국정밀공학회 학술발표대회 논문집, 21-22, 2020.](https://www.dbpia.co.kr/journal/articleDetail?nodeId=NODE10489383)
* 소스코드
    - [Project Github Pages - rootbKim/maxon_torque_cont](https://github.com/rootbKim/maxon_torque_cont)
* 영상
    - [Video - Gait Training Algorithm Based on Inverse Dynamics of Walking Rehabilitation Robot, DDgo Pro](https://youtu.be/AY8eiaZwY9s)

## 연구 기간

2019-12-01 ~ 2020-12-31

## 참여 인원

1명

## 기술 스택

- Embedded System(TI TMS320F28335)
- Motor Control(Maxon Motor Driver, Oriental BLDC Motor)
- Inverse Dynamics
- NX
- AutoCAD
- MATLAB
- Webot
- Recurdyn
- CATIA

## 세부 내용

#### DDgo Pro

* 연구에 사용된 보행재활로봇 DDgo Pro의 형상이며, 두 개의 5절링크와 한 개의 모터를 이용하여 보행재활훈련을 만든다.

<img src="/assets/img/portfolio/gait/ddgo_pro.jpg">

#### 5절 링크 메커니즘 및 엑츄에이터

* Inverse Dynamics를 해석한 보행재활로봇의 5절링크와 Freewheel을 이용한 병렬 구동 시스템이다.

<img src="/assets/img/portfolio/gait/actuator.png">

#### 제어시스템 구성도

- TI TMS230F28335를 이용한 제어
- 5절링크의 구동을 위한 1개의 BLDC 모터(오리엔탈)와 1개의 모터 제어기(Maxon)을 사용
- 탑승 및 하차 시 로봇을 정지시키기 위한 1개의 Magnetic Break와 1개의 모터 제어기
- 5절링크 메커니즘의 운동 상태를 관찰하기 위한 절대식 엔코더(Autonics)
- 로봇에 명령의 입력과 결과의 출력을 위한 Bluetooth 방식을 이용한 안드로이드 앱

<img src="/assets/img/portfolio/gait/control_system.png">

#### Newton-Euler 방식을 이용한 역동역학 해석

* 5절 링크의 Free-body diagram

<img src="/assets/img/portfolio/gait/free_body_diagram.png">

* 5절 링크의 좌표계 설정

<img src="/assets/img/portfolio/gait/five_link_mechanism.png">

* Newton-Euler 방식을 이용하여 다음과 같이 크랭크 관절의 위치에 따른 모터의 토크를 계산할 수 있었다.

<img src="/assets/img/portfolio/gait/newton_euler.png">

* 다음은 위의 식에 대한 결과로, 최종적으로 두 개의 5절링크의 구동을 위한 토크 프로파일을 다음과 같이 구할 수 있다.
    * 2km/h의 속도로 DDgo Pro에서 보행을 구현하기 위한 토크 프로파일
    * 자세한 역동역학 해석 과정은 [논문](https://doi.org/10.1007/s11370-021-00357-8)을 참조

<img src="/assets/img/portfolio/gait/crank_torque.jpg">


#### 3 가지 보행재활 모드

* 해석된 역동역학을 기반으로 세 가지 모드(Passive, Active-Assisted, Active)에 대한 동작과 그 효과에 대한 검증하였다.

<iframe width="840" height="315" src="https://www.youtube.com/embed/AY8eiaZwY9s" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

#### 근력보조성능 검증

* 세 가지 모드의 근육 활성도에 대한 비교 그래프로, 점진적인 훈련을 수행할 수 있음을 보여준다.

<img src="/assets/img/portfolio/gait/3mode_emg.png">

#### 환자 사용성 평가

* 보행 수준이 FAC(Functional Ambulation Category)가 3~4 단계인 뇌병변 환자 3명을 대상으로 실험을 진행하였다.

<img src="/assets/img/portfolio/gait/test_table.png">

* 보행 자세의 개선 여부를 측정하기 위하여 환측 다리와 건측 다리의 보행 모션 유사성을 측정하기 위하여 IMU 센서를 장착하고 실험을 진행하였다.

<img src="/assets/img/portfolio/gait/subjects.png">

* 보행 모션 유사성 검증은 다음의 상호 상관 관계 식을 이용하여 계산하였다.

<img src="/assets/img/portfolio/gait/cross_correlation.png">

* Active Assisted Mode와 Active Mode 를 대상으로 진행하였으며, Active Assisted Mode는 2.6km/h 목표 보행속도와 보조강도가 50%일 때, 보행을 하였고, Active Mode에서는 Active Assisted Mode에서 보행했던 속도로 각각 1분간 보행 훈련을 진행하였다. 훈련은 각 모드별로 5회씩 진행하였으며, 각 훈련마다 보행 유사도를 측정하여, 평균값을 비교하였다.

* 실험 결과는 다음과 같으며, 대부분 Active Assisted Mode의 상호상관계수가 높기 때문에, 구현된 Active Assisted Mode의 모터 보조가 보행 자세를 개선하는데 효과적임을 알 수 있다.
    * 1번 환자의 경우, 고관절에서만 모터 보조가 유효함을 보였으며, 2번, 3번 환자는 고관절과 슬관절에서 모두 모터 보조가 유효함을 보여주었는데, 1번 환자의 경우 인터뷰 자료를 토대로 분석한 결과 무릎의 힘이 다른 환자들보다 좋았기 때문에 이러한 결과가 나온 것으로 판단했다.

<img src="/assets/img/portfolio/gait/test_table.png">

* 다음은 각 환자의 결과 그래프이다.
    * 1번 환자
    <img src="/assets/img/portfolio/gait/test_graph1.png">
    * 2번 환자
    <img src="/assets/img/portfolio/gait/test_graph2.png">
    * 3번 환자
    <img src="/assets/img/portfolio/gait/test_graph3.png">

#### Conclusion

* 이 연구를 통해 DDgo Pro의 5절 링크가 사람이 보행 훈련을 할 수 있도록 사람의 보행 패턴에 맞는 움직임을 만들기 위한 5절 링크의 동역학 식을 계산하였다.
* 일반적인 매니퓰레이터 형태가 아닌, 병렬 구조의 링크를 Newton-Euler 방법을 이용하여 역동역학을 해석하였다.
* 계산된 역동역학 관계를 이용하여, 보행 훈련을 단계적으로 수행할 수 있도록 Passive Mode, Active-Assisted Mode, Active Mode를 구현하였고, 각 모드에 대한 효과를 검증하였다.
* 본 연구를 검증하기 위하여 3명의 뇌병변 환자를 대상으로 실험하였으며, 보행 자세 개선의 효과를 검증하였다.
* 본 연구를 통해 개발된 훈련 알고리즘은 기본의 보행 궤적만 추종하는 제어 알고리즘과는 다르게 사람의 의도 파악하여 제어를 함으로써 보행 궤적의 이질감을 줄이고, 보행 궤적에 유연하게 대처할 수 있으면서, 보행 궤적을 추종할 수 있도록 할 수 있는 보행 훈련을 생성할 수 있었다.
* 향후 연구의 방향을 다음과 같이 제시하였다.
    * 양쪽의 5절 링크를 하나의 모터로 제어하는 것이 아닌 두 개의 모터로 각각의 링크를 제어하는 것. 하나의 모터로 제어하게 되면, 양 발에 대한 독립적인 근력 보조가 불가능하기 때문.
    * 해당 연구는 환자의 보행 힘을 정확히 측정하기 위한 장치가 없었기 때문에 이를 가정하여 보상하였는데, 향후에는 발판에 힘 센서를 부착하여 환자의 보행 힘을 측정하여 정확하게 보상하는 것이 필요.