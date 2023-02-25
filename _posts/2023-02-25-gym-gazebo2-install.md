---
layout: post
title: gym-gazebo2 설치
feature-img: "assets/img/portfolio/alr_logo.png"
thumbnail: "assets/img/portfolio/alr_logo.png"
tags: [Gym Gazebo]
---

ROS2 환경에서 구동되는 gym-gazebo2의 설치 및 예제 에 대해서 정리한다.

## 1. Docker를 이용한 설치

우분투 및 ROS2 버전의 문제로, 직접 설치하여 사용하지 않고, 도커를 이용하여 설치한다. 설치 방법은 [gym-gazebo2 깃헙 페이지](https://github.com/AcutronicRobotics/gym-gazebo2/blob/dashing/docker/README.md)의 내용을 가져온 것이다.

#### gym-gazebo2 도커 이미지 설치

```bash
docker pull acutronicrobotics/gym-gazebo2
```

#### 도커 이미지 실행

```bash
# xhost 연결
xhost +

# 도커 실행
docker run --rm \
  -it \
  --name=gg2 \
  -h gym-gazebo2 \
  -v `pwd`:/tmp/gym-gazebo2 \
  --privileged \
  --gpus all \
  -e DISPLAY=:0 \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /etc/localtime:/etc/localtime:ro \
  -e TZ=Asia/Seoul \
  -v /dev:/dev \
  -w /root \
  acutronicrobotics/gym-gazebo2
```

위의 명령으로 docker가 실행되면, docker 내에서 다음의 명령을 수행한다.

> DISPLAY 환경변수는 내 PC의 DISPLAY 환경변수와 맞춰주면 된다. 그리고 `xhost +` 명령을 수행하고, docker를 실행시킨다.

```bash
cp -r /root/ros2_mara_ws /tmp/gym-gazebo2 #Inside the docker container, used to load visual models
```

## 2. MARA 예제 실행

`MARA`는 `Modular Articulated Robotic Arm`의 약자로 협동 로봇이며, ROS2 환경을 지원하는 로봇이다.

```bash
cd ~/gym-gazebo2/examples/MARA
python3 gg_random.py -g
```

## 참고문헌

- [gym-gazebo2 깃헙 페이지](https://github.com/AcutronicRobotics/gym-gazebo2)
