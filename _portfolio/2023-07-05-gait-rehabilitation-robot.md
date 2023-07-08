---
layout: post
title: 역동역학 기반의 파워어시스트 보행재활로봇 훈련 알고리즘 연구
feature-img: "assets/img/portfolio/gait/gait.jpg"
img: "assets/img/portfolio/gait/gait.jpg"
date: 05 July 2023
tags: [Gait Rehabilitation Robots]
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

* Inverse Dynamics를 해석한 보행재활로봇의 5절링크와 Freewheel을 이용한 병렬 구동 시스템

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

* 두 개의 5절링크의 구동을 위한 토크 프로파일
    * 2km/h의 속도로 DDgo Pro에서 보행을 구현하기 위한 토크 프로파일
    * 자세한 역동역학 해석 과정은 [논문](https://doi.org/10.1007/s11370-021-00357-8)을 참조

<img src="/assets/img/portfolio/gait/crank_torque.jpg">


#### 3 가지 보행재활 모드

* 해석된 역동역학을 기반으로 세 가지 모드(Passive, Active-Assisted, Active)에 대한 동작과 그 효과에 대한 검증

<iframe width="840" height="315" src="https://www.youtube.com/embed/AY8eiaZwY9s" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

#### 근력보조성능 검증

* 세 가지 모드의 근육 활성도에 대한 비교 그래프로, 점진적인 훈련을 수행할 수 있음을 나타냄

<img src="/assets/img/portfolio/gait/3mode_emg.png">
