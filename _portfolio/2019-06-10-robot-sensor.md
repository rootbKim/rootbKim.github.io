---
layout: post
title: 로봇센서응용 텀프로젝트 - balancing robot
feature-img: "assets/img/portfolio/sensor/sensor.png"
img: "assets/img/portfolio/sensor/sensor.png"
date: 10 June 2019
excerpt: 로봇센서응용 수업의 텀프로젝트로 수행한 balancing robot의 자세제어에 관한 프로젝트이다.
---

## 개요

로봇센서응용 수업의 텀프로젝트로 수행한 balancing robot의 자세제어에 관한 프로젝트로, 로봇의 자세제어, 위치제어, 모터제어를 수행하였다.

## 관련 링크

* [github repository](https://github.com/rootbKim/Balancing_robot)

## 연구 기간

2019-03-02 ~ 2019-06-10

## 참여 인원

2명

## 기술 스택

- Arduino
- Control Theory
- Modeling
- IMU
- Bluetooth

## 세부 내용

#### 자세제어

* PID를 이용해 제자리에서 자세를 제어한 영상이다.

<img src="/assets/img/portfolio/sensor/position_control.gif">

#### 위치 제어

* 로봇이 10초 마다 일정 거리를 이동하도록 제어한 영상이다.

<img src="/assets/img/portfolio/sensor/distance_control.gif">

#### 방향 제어 및 위치 제어

* 로봇을 블루통신을 이용해서, 일정 거리만큼 이동하고, 방향을 회전하도록 제어하는 영상이다.

<img src="/assets/img/portfolio/sensor/final_control.gif">

#### Conclusion

* 수업을 진행하면서, 로봇에 필요한 시스템을 설계하고, 아두이노, IMU, 엔코더, 모터 등에 대한 단위 테스트를 수행하였다.
* balacing 로봇의 모델을 해석하고, 이를 기반으로 제어 시스템을 구축하고, 이를 기반으로 로봇의 자세제어, 위치제어, 방향제어를 수행하였다.
* 완벽한 제어가 되지는 못했지만, 간단한 시스템으로 로봇의 다양한 제어를 수행해볼 수 있었다.