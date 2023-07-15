---
layout: post
title: 2족보행 로봇의 보행 알고리즘 시뮬레이션
feature-img: "assets/img/portfolio/biped/biped.png"
img: "assets/img/portfolio/biped/biped.png"
date: 08 July 2023
tags: [Biped Walking Robot]
excerpt: 2족 보행로봇인 SUBO3의 시뮬레이션 환경을 구축하고, CTC 기반의 보행 알고리즘을 위한 RBDL 라이브러리를 이용한 6자유도 제어 및 로봇의 자세 제어에 대한 연구 내용이다.
---

## 개요

하나의 다리에 6개의 자유도를 가진 2족 보행로봇의 GAZEBO 시뮬레이션 환경을 구성하고, 로봇의 각 관절을 제어하기 위해 RBDL 라이브러리를 이용한 CTC 제어 알고리즘을 구성한다. 이를 통해 로봇의 자세 제어와 제자리 보행 방법을 연구한다.

## 관련 링크

* 소스코드
    - [Project Github Pages - subo3 pkages](https://github.com/rootbKim/subo3_pkgs)
    - [Project Github Pages - SUBO3 urdef](https://github.com/rootbKim/SUBO3)
    - [Project Github Pages - [TEST] planar robots](https://github.com/rootbKim/planar_robot_pkgs)
* 영상
    - [Test Video](https://github.com/rootbKim/subo3_pkgs/blob/master/video/video.gif)

## 연구 기간

2020-12-01 ~ 2021-05-01

## 기술 스택

- ROS(kinetic)
- GAZEBO
- Rigid Body Dynamics Library(RBDL)
- Eigen Library
- SolidWorks
- Computed Torque Control(CTC)
- MATLAB
- Mathematica

## 세부 내용

#### SUBO3

* SUBO3는 HRRLAB에서 연구 중이던 2족 보행 로봇으로, 2족 보행 로봇의 보행 알고리즘을 연구하기 위해 개발 및 연구에 사용된 로봇이다.

<img src="/assets/img/portfolio/biped/subo3.png" height="500">

#### GAZEBO 환경 구성

* SUBO3의 3D 모델(SolidWorks)을 이용하여 URDF 파일을 만들고, 이를 ROS(kinetic) GAZEBO 환경을 구축하였다.

<img src="/assets/img/portfolio/biped/solidworks.png" height="500">

* GAZEBO 환경에서의 구조는 `GAZEBO plugin` 형태로 library를 만들어, GAZEBO 실행 시 해당 plugin이 함께 실행되는 형태이다. plugin은 각 관절에 필요한 토크를 계산하고, GAZEBO 상의 모터에 토크를 출력하며, 미리 정의된 msg 인터페이스 패키지를 이용하여 로봇에 제어명령을 내린다. 다음은 해당 프로젝트의 플로우 차트이다.

<img src="/assets/img/portfolio/biped/flowchart.png">

* `Load`: GAZEBO 실험 환경을 세팅하는 단계이다.
    * `RBDL INIT`: RBDL 라이브러리를 초기화하고 각 물성치, 조인트 설정, RBDL에 사용되는 변수를 초기화. 왼발 / 오른발의 골반 모델, 양발 지지 모델, 한발 지지 모델을 생성하는 단계.
    * `GetLinks`, `GetJoints`: SDF 파일로부터 링크와 조인트를 가져옴
    * `InitROSPub`: ROS topic을 초기화
    * `SensorSetting`: IMU 센서 설정
    * `Set Joint Var.`: 각 관절에 입력할 joint torque를 계산하여 저장할 변수 선언
* `UpdateAlgorithm`: GAZEBO 실험 환경을 관측하고, 제어에 필요한 계산을 수행 및 명령하는 단계이다.
    * `IMUSensorRead`: IMU 센서로부터 데이터를 읽음
    * `FTSensorRead`: FT 센서로부터 각 관절의 FT 센서 데이터를 읽음
    * `EncoderRead`: 각 조인트의 각도를 읽음
    * `RBDL variable update`: 측정된 데이터를 이용하여 RBDL 모델의 각도, 각속도, 각가속도를 업데이트
    * `Calc ZMP`: FT 센서의 데이터를 이용한 ZMP 위치 계산
    * `Pos Generation`: 로봇이 수행할 자세를 설정. 자세에 따른 PD 제어 또는 CTC를 통하여 모션 수행을 위한 각 조인트 토크를 계산
    * `jointController`: 계산된 토크값을 각 조인트에 입력
    * `ROSPub & Print`: 결과에 대한 메시지 Pub 및 데이터 출력

#### [RBDL](https://rbdl.github.io/)

* RBDL이란 강체 동역학을 계산하지 않고, 함수들을 사용하여 효율적으로 알고리즘을 구현할 수 있는 라이브러리를 뜻한다.

* 아래와 같은 Computed Torque Control 시스템을 구현하기 위해 각 부분에서 사용해야 할 RBDL Function을 정리한 것이다.

<img src="/assets/img/portfolio/biped/CTC.png">

* CTC 제어를 위한 flow는 다음과 같다.
    1. 골반 중심과 발 중심의 목표 위치 설정
    2. 현재 로봇의 위치를 Feedback 받아 목표 위치와의 오차 계산
    <img src="/assets/img/portfolio/biped/CTC_POS.png">
    3. CTC를 이용하여 오차에 대한 토크 계산
    <img src="/assets/img/portfolio/biped/CTC_TORQUE.png">

* 이 CTC 시스템을 RBDL을 이용하여 발 중심의 CTC 계산, 골반 중심의 CTC 계산, 한발 중심의 CTC를 계산하였다.

#### ZMP를 이용한 CTC 모드 변환

* 다음과 같이 ZMP(Zero Moment Point)를 구한다. ZMP는 양발을 지지하고 있을 때는 양 발의 사이에 위치하나, 한쪽 발을 들어올리면 반대쪽 발로 ZMP가 이동하는데, ZMP가 발을 벗어나게 되면 넘어지게 된다.

<img src="/assets/img/portfolio/biped/ZMP.png">

* ZMP를 이용한 골반 중심의 CTC와 발 중심의 CTC, 한발 중심의 CTC를 조합하여, ZMP 상태에 따라 천이하도록 하였다.
    * ZMP의 위치에 따라 로봇이 양발 지지 중인지, 한쪽 발로 지지중인지를 판단하여, 현재의 로봇 자세에 대한 각 CTC를 적절히 조합하여 토크를 계산하였다.
    * 세 가지의 CTC 모드를 사용한 이유는 각 CTC 모드 별로 자세에 따라 장점이 존재하는데, 이 장점을 구간별로 극대화하여 사용하기 위함이다.
<img src="/assets/img/portfolio/biped/ZMP_CTC.png">\
<img src="/assets/img/portfolio/biped/ZMP_CTC2.png">

#### 제자리 보행 테스트

이 프로젝트는 최종적으로 2족 보행의 제자리 걸음시 토크 제어를 이용한 자세 유지와, 지면의 장애물을 밟았을 때 안정적으로 자세를 유지하며 제자리 보행을 지속할 수 있도록 자세 제어를 실시하였다. 실험 영상은 [영상 링크](https://github.com/rootbKim/subo3_pkgs/blob/master/video/video.gif)에서 확인할 수 있다.

#### Conclusion

* 이 실험을 통해 2족보행 로봇 SUBO의 GAZEBO 환경을 구축하고, RBDL 라이브러리를 이용한 토크 제어의 환경을 구축할 수 있었다.
* RBDL을 이용한 로봇동역학 계산을 수행하면서, 직접 계산한 기구학과 RBDL을 이용한 기구학을 비교하며, RBDL을 이용한 해석을 검증하였다.
* 로봇의 각 좌표계의 설정과, GAZEBO에서의 좌표계 변환, RBDL의 좌표계 간의 변환 관계를 해석하는 데 어려움이 있었으나, 실험을 통해 극복할 수 있었다.
* 2족보행 로봇의 토크 제어를 위해 골반 중심 CTC, 발 중심의 CTC를 구현하며, 다관절 로봇의 동역학 제어를 수행하였고, 나아가 로봇의 제자리 보행을 구현하였다.
* 구현된 RBDL을 이용한 CTC를 이용하여 ZMP에 따른 2족보행 로봇의 제자리 보행을 안정화하여, 제자리 보행 중 외란에 대한 극복과 장애물을 밟았을 때도 자세를 유지할 수 있도록 제어하였다.