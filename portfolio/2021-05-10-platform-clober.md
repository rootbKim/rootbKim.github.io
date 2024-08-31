---
layout: portfolio
title: 연구용 로봇 플랫폼의 ROS 오픈소스 패키지
feature-img: "/assets/img/portfolio/clober/clober.png"
img: "/assets/img/portfolio/clober/clober.png"
date: 10 May 2021
excerpt: CLOBOT의 연구용 로봇 플랫폼으로 연구된 clober의 ROS / ROS2 환경의 오픈 소스 패키지 소프트웨어의 구성과 개발 및 컨트리뷰팅
---

# 개요

모바일 로봇의 베이스 플랫폼이 되는 `clober`라는 로봇의 ROS / ROS2 환경을 구축하고, 해당 환경에서 필요한 기본 패키지들을 구성하여 오픈 소스화하는 프로젝트이다.

# 관련 링크

* 소스코드
    - [CLOBOT-Co-Ltd / clober](https://github.com/CLOBOT-Co-Ltd/clober)
* 영상
    - [RMF based multi-robot traffic management application use case](https://vimeo.com/649654300)

# 프로젝트 기간

2021-05-10 ~ 2021-10-06

# 개발 참여 인원 / 담당 역할

3명 / ROS2 Nav2 기능 분석 및 관련 패키지 구현, GAZEBO 시뮬레이션 패키지 구현 

# 기술 스택

- Open Source Contributing
- ROS(noetic)
- ROS2(foxy)
- GAZEBO
- Nav2
- Git

# 세부 내용

### CLOBER

* `clober`는 모바일 로봇의 베이스 플랫폼이 되는 로봇으로, 해당 로봇 위에 서빙 플랫폼, 호텔 플랫폼 등의 플랫폼을 올릴 수 있는 구조로 되어있다. 따라서 `clober`는 로봇의 움직임을 담당하기 때문에 기본적인 모터 구동과 Navigation 관련 패키지들로 구성하여 배포를 하였다.

<img src="/assets/img/portfolio/clober/clober.png">

### 패키지 구성

* `clober`의 패키지 구성은 ROS 개발 키트로 유명한 [`tutlebot3`](https://github.com/ROBOTIS-GIT/turtlebot3)를 많이 참조하여 구성하였다.
* `clober`의 패키지는 bringup, description, navigation, simulation, slam과 같은 기본적인 구성으로 이루어졌다.
* 또한 `clober`는 ROS noetic 버전과 ROS2 foxy 버전을 지원하도록 하였다.

### release

* ROS 패키지를 배포하기 위하여 [`rosdistro`](https://github.com/ros/rosdistro)에 `Pull Request`를 하여 누구든지 패키지로 설치를 할 수 있도록 하였다.
* 버전을 release 하기 위해 버전 관리를 수행하였다.

### Conclusion

* 이 프로젝트를 통해 Git을 이용한 오픈 소스 패키지를 구성하고 컨트리뷰팅을 경험할 수 있었다.
* 오픈 소스 프로젝트를 통해서 오픈 소스에 대한 이해와 Git을 활용한 Issue 관리 및 Pull Request, 버전관리와 release를 경험해 볼 수 있었다.
* ROS2 foxy 버전을 사용함으로써, ROS2와 Nav2에 대해 스터디할 수 있었다.