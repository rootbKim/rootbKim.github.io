---
layout: note_page
title: Docker Cycle
tags: [Docker]
category: "Dev"
---

Docker 이미지의 관리 방법과 DockerFile의 사용방법을 정리한다.

# 1. Docker 이미지 만들기

## 1.1 기본 이미지를 이용한 새로운 컨테이너 만들기

Docker 이미지를 만들기 위해 우분투 20.04 기본 이미지를 가져와, ROS2 Foxy 환경을 직접 설치한 새로운 컨테이너를 만들어 볼 것이다.

먼저 `pull` 명령어로 우분투 20.04 버전 이미지를 설치한다.

```bash
docker pull ubuntu:20.04
```

설치가 완료되면 `images` 명령어로 새롭게 설치된 ubuntu:20.04 이미지를 확인할 수 있다.

```bash
docker images
```

다른 ubuntu:20.04와의 구분을 위해 pull 받은 image의 이름과 태그를 수정한다.

```bash
docker image tag ubuntu:20.04 ubuntu:foxy
```

이제 기본 ubuntu 20.04 환경에 새로운 컨테이너를 만들기 위해 설정한 ubuntu:20.04-ros-foxy 이미지를 실행시킨다.

```bash
docker run -it --name ubuntu_foxy ubuntu:foxy
```

> `-it` 명령으로 실행시켜 터미널에서 해당 환경을 사용할 수 있도록 하였다.

> `--name` 명령으로 해당 컨테이너가 `ubuntu_foxy`라는 이름으로 실행되도록 하였다.

docker를 run 하면 실행이 되는데, docker의 실행 상태 확인은 다음의 명령으로 확인할 수 있다.

```bash
docker ps
```

이제 실행된 도커 터미널에서 ros2 foxy 버전을 설치하도록 한다. 설치 방법은 [ROS2 Foxy 공식 홈페이지](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)를 참조한다.

> 새로 설치된 우분투 환경에서 `sudo` 권한을 가지려면 `apt-get update && apt-get install -y sudo`으로 sudo 설치 및 사용자 설정을 수행한다.

> 설치 시간이 꽤 소요될 것이다. ROS에서도 docker 이미지를 제공하므로, 해당 이미지를 받아서 작업하면 편하지만, 지금은 이미지 작업의 이해를 돕기 위해 해당 과정을 거치므로, 시간이 너무 소요가 된다면 설치 과정은 중간에 생략해도 좋다.

## 1.2 커밋(Commit)

원하는 작업이 container 안에서 완료가 되면 현재까지 작업된 내용을 커밋하여 새로운 이미지로 생성하는 단계이다. 명령어는 다음과 같다.

```bash
docker commit ubuntu_foxy ubuntu:foxy_1
```

> 첫 번째 인자로 현재 실행 중인 컨테이너 명을 입력하고, 두 번째 인자로 `[이미지명]:[태그]`를 입력한다.

# 2. Docker Hub과 Push Pull

## 2.1 Docker Hub

Docker Hub([https://hub.docker.com/](https://hub.docker.com/))는 GitHub나 Bitbucket과 같은 소스코드 관리 툴과 연계하여 코드를 빌드하는 기능이나 실행 가능한 애플리케이션의 이미지를 관리하는 Docker의 공식 리포지토리 서비스이다. 즉, Docker Hub에 내가 만든 이미지를 등록하여 관리하고, 필요할 때 다운받을 수 있는 원격 저장소 역할을 하며, 해당 원격 저장소를 이용하여 다른 사람과 Docker 이미지를 공유할 수 있고, 다른 사람의 이미지를 이용할 수도 있으며, 버전 관리에 유용하게 사용할 수 있다. 단, Docker Hub는 개인 대상이 아닌 기업 대상으로는 유료화를 하기 때문에 사용에 유의해야 한다.

Docker Hub를 사용하면 먼저 계정을 만들고 진행한다.

## 2.2 푸쉬(Push)

Docker Hub의 개인 계정에 푸쉬를 하기 위해 로그인을 진행한다.

```bash
docker login -u [계정이름]
```

> 로그인 정보는 `docker info | grep Username`으로 확인하고, 로그 아웃은 `docker logout`을 이용한다.

Docker Hub의 개인 계정에 푸쉬하기 위해서는 이미지가 `[계정이름]/[이미지 이름]:[태그]` 형식이 되어야 한다. 이에 따라 생성한 이미지를 변경한다.

```bash
docker tag ubuntu:foxy_1 [계정이름]/ubuntu:foxy_1
```

> 이미지 이름은 처음 생성할 때 부터 [계정이름]을 붙여서 만들어도 된다.

이제 로그인 된 계정을 이용하여 지금까지 만든 이미지를 push 한다.

```bash
docker push [계정이름]/ubuntu:foxy_1
```

푸쉬가 다 되면 Docker Hub의 Repository에서 푸쉬한 이미지를 확인한다.

## 2.3 풀(Pull)

다음으로 Docker Hub에 저장된 이미지를 가져오기 위해서는 계정 이름과, 가져오고자 하는 이미지 명, 태그를 이용하면 된다.

```bash
docker pull [계정이름]/ubuntu:foxy_1
```

# 3. DcokerFile을 이용한 이미지 빌드하기

## 3.1 DockerFile과 빌드란?

`도커 파일`(DockerFile)은 docker 에서 이미지를 생성하기 위한 용도로 작성하는 스크립트 파일로, 만들고자 하는 이미지에 대한 정보를 기술해 둔 템플릿(template)이라고 볼 수 있다.

DockerFile을 이용하여 이미지를 빌드하는 것의 장점은 다음과 같다.

- 도커 파일에 이미지의 빌드를 위한 스크립트가 작성되어 있으므로, 이미지가 어떻게 만들어졌는지를 기록한다.
- 이미지로 배포하는 것이 아닌 도커파일만 있으면 배포를 용이하게 할 수 있다.
- 컨테이너 실행 시 특정 명령을 수행하도록 작성할 수 있다.

## 3.2 DockerFile

DockerFile에 사용되는 기본적인 지시어가 정리된 내용이다. 해당 내용은 기본이 되는 내용이며, 추가적인 기능이나 각 기능의 자세한 내용은 [Docker 문서](https://docs.docker.com/engine/reference/builder/)를 참조한다.

- FROM : 베이스 이미지를 지정하는 지시어

  > 어느 이미지에서 시작할건지를 의미한다.

- MAINTAINER : 이미지를 생성한 개발자의 정보 (1.13.0 이후 사용 X)

- LABEL : 이미지에 메타데이터를 추가 (key-value 형태)

- RUN : 새로운 레이어에서 명령어를 실행하고, 새로운 이미지를 생성한다.

  > RUN 명령을 실행할 때 마다 레이어가 생성되고 캐시된다. 따라서 RUN 명령을 따로 실행하면 apt-get update는 다시 실행되지 않아서 최신 패키지를 설치할 수 없다. RUN 명령 하나에 apt-get update와 install을 함께 실행 해야한다.

- WORKDIR : 작업 디렉토리를 지정한다. 해당 디렉토리가 없으면 새로 생성한다.

  > 작업 디렉토리를 지정하면 그 이후 명령어는 해당 디렉토리를 기준으로 동작한다. cd 명령어와 동일하다.

- EXPOSE : Dockerfile의 빌드로 생성된 이미지에서 열어줄 포트를 의미한다.

  > 호스트 머신과 컨테이너의 포트 매핑시에 사용된다. 컨테이너 생성 시 -p 옵션의 컨테이너 포트 값으로 EXPOSE 값을 적어야한다.

- USER : 이미지를 어떤 계정에서 실행 하는지 지정

  > 기본적으로 root에서 해준다.

- COPY / ADD : build 명령 중간에 호스트의 파일 또는 폴더를 이미지에 가져오는 것

  > ADD 명령문은 좀 더 파워풀한 COPY 명령문이라고 생각할 수 있다. ADD 명령문은 일반 파일 뿐만 아니라 압축 파일이나 네트워크 상의 파일도 사용할 수 있다. 이렇게 특수한 파일을 다루는 게 아니라면 COPY 명령문을 사용하는 것이 권장된다.

- ENV : 이미지에서 사용할 환경 변수 값을 지정한다.

  > path 등

- CMD / ENTRYPOINT : 컨테이너를 생성,실행 할 때 실행할 명령어

  > docker run 명령으로 컨테이너를 생성하거나, docker start 명령으로 정지된 컨테이너를 시작할 때 실행된다. 보통 컨테이너 내부에서 항상 돌아가야하는 서버를 띄울 때 사용한다.

- CMD : CMD는 docker run 실행 시, 추가적인 명령어에 따라 설정한 명령어를 수정하고자 할 때 사용된다.

  - CMD 명령은 3가지 형태가 있다.

  ```bash
  CMD [“executable”,”param1”,”param2”]
  CMD [“param1”,”param2”]
  CMD command param1 param2
  ```

- ENTRYPOINT : docker run 실행 시, 추가적인 명령어의 존재 여부와 상관 없이 무조건 실행되는 명령이다.

  - ENTRYPOINT 명령은 2가지 형태가 있다.

  ```bash
  ENTRYPOINT [“executable”, “param1”, “param2”]
  ENTRYPOINT command param1 param2
  ```

- ARG : Argument 설정

## 3.3 DockerFile 예시

다음은 ROS Foxy 버전에서 제공하는 Dockerfile의 예시이다.

```bash
ARG FROM_IMAGE=ros:foxy
ARG OVERLAY_WS=/opt/ros/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# clone overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
RUN echo "\
repositories: \n\
  ros2/demos: \n\
    type: git \n\
    url: https://github.com/ros2/demos.git \n\
    version: ${ROS_DISTRO} \n\
" > ../overlay.repos
RUN vcs import ./ < ../overlay.repos

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# multi-stage for building
FROM $FROM_IMAGE AS builder

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -y \
      --from-paths \
        src/ros2/demos/demo_nodes_cpp \
        src/ros2/demos/demo_nodes_py \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
ARG OVERLAY_MIXINS="release"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --packages-select \
        demo_nodes_cpp \
        demo_nodes_py \
      --mixin $OVERLAY_MIXINS

# source entrypoint setup
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

# run launch file
CMD ["ros2", "launch", "demo_nodes_cpp", "talker_listener.launch.py"]
```

## 3.4 생성한 Dockerfile을 Image로 빌드

빌드 명령은 기본적으로 다음과 같은 형태이다.

```bash
docker build [OPTIONS] PATH | URL | -
```

-t 옵션을 이용하면 빌드시 생성되는 이미지의 이름을 지정할 수 있는데, 그 현재 경로에 위에서 만든 `Dockerfile`을 저장하고, ros-foxy라는 이름으로 빌드하는 경우 그 예는 다음과 같다.

```bash
docker build -t ros-foxy .
```

빌드가 완료되고, image를 확인하면 `ros-foxy:latest`라는 새로운 이미지가 생성된 것을 확인할 수 있다.

# 참고문헌

- [[Docker] 개념 정리 및 사용방법까지](https://cultivo-hy.github.io/docker/image/usage/2019/03/14/Docker%EC%A0%95%EB%A6%AC/)
- [도커와 컨테이너의 이해 (3/3) - Docker image, Dockerfile, Docker compose](https://tech.cloudmt.co.kr/2022/06/29/%EB%8F%84%EC%BB%A4%EC%99%80-%EC%BB%A8%ED%85%8C%EC%9D%B4%EB%84%88%EC%9D%98-%EC%9D%B4%ED%95%B4-3-3-docker-image-dockerfile-docker-compose/)
- [(Docker)도커 - 도커 build, commit 명령어로 이미지 생성하기](https://reddb.tistory.com/181)
- [[Docker] docker commit 명령어 사용하는 방법!](https://somjang.tistory.com/entry/Docker-docker-commit-%EB%AA%85%EB%A0%B9%EC%96%B4-%EC%82%AC%EC%9A%A9%ED%95%98%EB%8A%94-%EB%B0%A9%EB%B2%95)
- [[Docker] 내가 만든 Container를 image로 만들자!(commit, build, Dockerfile 활용하기)](https://techblog-history-younghunjo1.tistory.com/235)
- [[Docker] Dockerfile 개념 및 작성법](https://wooono.tistory.com/123)
- [docker :: 도커파일(Dockerfile) 의 개념, 작성 방법/문법, 작성 예시](https://toramko.tistory.com/entry/docker-%EB%8F%84%EC%BB%A4%ED%8C%8C%EC%9D%BCDockerfile-%EC%9D%98-%EA%B0%9C%EB%85%90-%EC%9E%91%EC%84%B1-%EB%B0%A9%EB%B2%95%EB%AC%B8%EB%B2%95-%EC%9E%91%EC%84%B1-%EC%98%88%EC%8B%9C)
