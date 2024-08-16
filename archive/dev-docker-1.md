---
layout: note_page
title: Docker 개념과 기본 명령어
tags: [Docker]
category: "Dev"
---

Docker의 기본 개념과 설치, 그리고 기본 명령어에 대해 정리한다.

# 1. Docker 개념

## 1.1 Docker

[도커(Docker)](https://www.docker.com/)는 컨테이너 기반의 오픈소스 가상화 플랫폼으로, 컨테이너를 관리하는 툴이다. 도커를 사용하는 가장 큰 이유는 다양한 프로그램, 실행환경을 컨테이너로 추상화하고 동일한 인터페이스를 제공하여 프로그램의 배포 및 관리를 쉽게하기 위해서이다.

## 1.2 Docker를 사용해야 하는 이유

1. `구성 단순화`: Docker는 하나의 Configuration으로 모든 플랫폼에서 실행할 수 있습니다. Configuration 파일을 코드에 넣고 환경 변수를 전달하여 다른 환경에 맞출 수 있습니다. 따라서 하나의 Docker 이미지를 다른 환경에서 사용할 수 있습니다.
2. `코드 관리`: Docker는 일관된 환경을 제공하여 개발 및 코딩을 훨씬 편안하게 만들어줍니다. Docker 이미지는 변경이 불가하기에 개발환경에서 운영 환경까지 애플리케이션 환경이 변경되지 않는 이점이 존재합니다.
3. `개발 생산성 향상`: 개발 환경을 운영 환경에 최대한 가깝게 복제할 수 있습니다. Docker를 사용하면 코드가 운영 환경의 컨테이너에서 실행될 수 있으며 VM과 달리 Docker는 오버 헤드 메모리 용량이 적기에 여러 서비스를 실행하는데 도움이 됩니다. 또한 Docker의 Shared Volume을 사용하여 호스트에서 컨테이너의 어플리케이션 코드를 사용할 수 있도록 할 수 있습니다. 이를 통해 개발자는 자신의 플랫폼 및 편집기에서 소스 코드를 편집할 수 있으며 이는 Docker내에서 실행 중인 환경에 반영됩니다.
4. `애플리케이션 격리`: Web Server(e.g. Apache, Nginx)와 연결된 API 서버를 격리할 필요가 있는 경우가 있습니다. 이 경우 다른 컨테이너에서 API를 서버를 실행할 수 있습니다.
5. `빠른 배포`: 컨테이너가 OS를 부팅하지 않고 어플리케이션을 실행하기 때문에 Docker 컨테이너를 매우 빠르게 만들 수 있습니다.

## 1.3 Docker 사용 예

- 개발자는 코드를 로컬로 작성하고 Docker 컨테이너를 사용하여 동료와 작업을 공유할 수 있다.
- Docker를 사용하여 애플리케이션을 테스트 환경으로 푸시하고 자동 및 수동 테스트를 수행할 수 있다.
- 개발자는 버그를 발견하면 개발 환경에서 오류를 수정하고 테스트 및 검증을 위해 테스트 환경에 다시 배치할 수 있다.
- 테스트가 완료되면 갱신된 이미지를 실전 가동 환경에 푸시하는 것만으로 고객에게 수정을 제공할 수 있다.

## 1.4 Docker 아키텍처

Docker는 `클라이언트-서버 아키텍처`를 사용한다.`도커 클라이언트`는 `도커 데몬`과 통신합니다. 도커 데몬은 `도커 컨테이너`의 빌드, 실행 및 배포 작업을 주로 수행한다. 도커 클라이언트와 데몬은 동일한 시스템에서 실행하거나 도커 클라이언트를 원격 도커 데몬에 연결할 수 있다. 도커 클라이언트와 데몬은 REST API, UNIX 소켓 또는 네트워크 인터페이스를 사용하여 통신한다. 또 다른 도커 클라이언트는 컨테이너 세트로 구성된 응용 프로그램을 사용할 수 있는 도커 구성이다.

<img src="/assets/img/posts/230208_docker_architecture.svg">

### Docker Daemon

`도커 데몬(dockerd)`는 Docker API 요청을 수신하고 이미지, 컨테이너, 네트워크, 볼륨 등의 **Docker 개체를 관리**한다. 데몬은 **다른 데몬과 통신하여 도커 서비스를 관리**할 수도 있다.

### Docker Client

`도커 클라이언트(docker)`는 **많은 Docker 사용자가 Docker와 대화하는 주요 방법**이다. 클라이언트는 `docker run` 명령어를 도커 데몬에 송신하여, 도커 데몬을 실행한다. `docker run` 명령어는 Docker API를 사용하며, 도커 클라이언트는 여러 데몬과 통신할 수 있다.

### Docker Desktop

`도커 데스크톱(Docker Desktop)`은 Mac, Windows 또는 Linux 환경에서 쉽게 설치할 수 있는 애플리케이션으로 컨테이너형 애플리케이션과 마이크로 서비스를 구축 및 공유할 수 있다. 도커 데스크톱에는 Docker Daemon, Docker Client, Docker Compose, Docker Content Trust, Kubernetes 및 자격 정보 도우미 등을 포함한다.

### Docker Registries

`도커 레지스트리(Docker Registries)`에 도커 이미지가 저장된다. Docker Hub는 누구나 사용할 수 있는 퍼블릭 레지스트리이며, Docker는 기본적으로 [Docker Hub](https://hub.docker.com/)에서 이미지를 검색하도록 구성되어 있으며, 독자적인 프라이빗 레지스트리를 이용할 수도 있다.

`docker pull` 또는 `docker run` 명령을 이용하여 이미지를 등록된 레지스트리에서 가져온다. 반대로 `docker push` 명령을 실행하면 이미지가 설정된 레지스트리에 푸시할 수 있다.

### Docker Objects

도커를 사용하면 `이미지(Image)`, `컨테이너(Container)`, 네트워크, 볼륨, 플러그인 및 기타 개체를 만들고 사용하게 된다. 여기서는 중요하면서 헷갈리기 쉬운 이미지와 컨테이너에 대해서 조금 더 자세히 알아본다.

#### Image

이미지(Image)는 컨테이너 실행에 필요한 파일과 설정값등을 포함하고 있는 것으로 상태값을 가지지 않고 변하지 않는다(Immutable). 이러한 이미지는 대부분 다른 이미지를 기반으로 하며 커스터마이즈를 하여 사용하게 된다.

이렇게 독자적인 이미지를 작성할 수도 있고, 다른 사람이 만들어 레지스트리에 게시한 이미지만 사용할 수도 있다. 자신의 이미지를 빌드하려면 이미지를 만들고 실행하는 데 필요한 단계를 정의하기 위한 간단한 구문을 사용하여 도커 파일을 만들고, 도커 파일의 각 명령은 이미지에 `레이어`를 생성한다. 도커 파일을 변경하고 이미지를 재구축하면 변경된 레이어만 재구축된다.

> 이미지는 컨테이너를 실행하기 위한 모든 정보를 가지고 있기 때문에 보통 용량이 크다. 처음 이미지를 다운받을 땐 크게 부담이 안되지만 기존 이미지에 파일 하나 추가했다고 큰 용량의 이미지를 다시 다운받는 것은 비효율적이다. 도커는 이런 문제를 해결하기 위해 `레이어(Layer)`라는 개념을 사용하고 유니온 파일 시스템을 이용하여 여러 개의 레이어를 하나의 파일시스템으로 사용할 수 있게 해준다. 이미지는 **여러 개의 읽기 전용 read only 레이어로 구성되**고 파일이 추가되거나 수정되면 **새로운 레이어가 생성**된다.

> 이미지는 `url` 방식으로 관리하며 태그를 붙일 수 있다. `ubuntu 14.04` 이미지는 `docker.io/library/ubuntu:14.04` 또는 `docker.io/library/ubuntu:trusty` 이고 `docker.io/library`는 생략가능하여 `ubuntu:14.04` 로 사용할 수 있다. 이러한 방식은 이해하기 쉽고 편리하게 사용할 수 있으며 태그 기능을 잘 이용하면 테스트나 롤백도 쉽게 할 수 있다.

#### Container

컨테이너(Container)는 이미지의 실행 가능한 인스턴스로 개별 Software의 실행에 필요한 실행환경을 독립적으로 운용할 수 있도록 기반환경 또는 다른 실행환경과의 간섭을 막고 **실행의 독립성을 확보해주는 운영체계 수준의 격리 기술**을 말한다.

Docker API 또는 CLI를 사용하여 컨테이너를 생성, 시작, 중지, 이동 또는 삭제할 수 있습니다.컨테이너를 하나 이상의 네트워크에 연결하거나, 컨테이너에 스토리지를 연결하거나, 컨테이너의 현재 상태를 기반으로 새 이미지를 생성할 수도 있습니다.

컨테이너는 이미지를 실행한 상태라고 볼 수 있고 추가되거나 변하는 값은 컨테이너에 저장된다. 즉, **실행된 컨테이너 내에서 변경사항이 발생하면 이미지에는 영향이 없으며, 컨테이너 내에서만 변경사항이 적용**된다.

또한 **이미지 하나로 여러개의 컨테이너를 생성**할 수 있고 컨테이너의 상태가 바뀌거나 컨테이너가 삭제되더라도 이미지는 변하지 않고 그대로 남아있는다.

기본적으로 컨테이너는 다른 컨테이너 및 해당 호스트 시스템과 비교적 잘 격리된다. 컨테이너의 네트워크, 스토리지 또는 기타 기본 하위 시스템이 다른 컨테이너 또는 호스트 시스템에서 격리되는 정도를 제어할 수 있다.

컨테이너는 이미지 및 컨테이너를 생성하거나 시작할 때 제공하는 구성 옵션에 의해 정의된다. 컨테이너를 제거하면 영구 스토리지에 저장되지 않은 상태의 변경 사항이 사라진다.

# 2. Docker 설치

- [Docker 설치 페이지](https://docs.docker.com/engine/install/ubuntu/)

```bash
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```

설치
```bash
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

설치 확인하기

```bash
sudo docker run hello-world
```

만약에 sudo 권한 없이 사용이 안된다면, 다음과 같이 user 그룹을 추가해준다.

```bash
sudo usermod -aG docker $USER
```

# 3. Docker 기본 명령어

- 실행 중인 컨테이너 목록 확인하기 (ps)

```bash
docker ps [OPTIONS]
```

- 컨테이너 중지하기 (stop)

```bash
docker stop [OPTIONS] CONTAINER [CONTAINER...]
```

> 도커 ID의 전체 길이는 64자리이나, 명령어의 인자로 전달할 때는 전부 입력하지 않아도 된다. 예를 들어 ID가 abcdefgh…라면 abcd만 입력해도 된다.

- 전체 컨테이너 조회하기

```bash
docker container ls -a
```

- 컨테이너 제거하기 (rm) : 종료된 컨테이너를 완전히 제거하는 명령

```bash
docker rm [OPTIONS] CONTAINER [CONTAINER...]
```

- 이미지 목록 확인하기 (images)

```bash
docker images [OPTIONS] [REPOSITORY[:TAG]]
```

- 이미지 다운로드하기 (pull)

```bash
docker pull [OPTIONS] NAME[:TAG|@DIGEST]
```

- 이미지 삭제하기 (rmi)

```bash
docker rmi [OPTIONS] IMAGE [IMAGE...]
```

> images 명령어를 통해 얻은 이미지 목록에서 이미지 ID를 입력하면 삭제된다. 단, 컨테이너가 실행중인 이미지는 삭제되지 않는다.

- 컨테이너 로그 보기 (logs) : 컨테이너가 정상적으로 동작하는지 확인하기 위함

```bash
docker logs [OPTIONS] CONTAINER
```

- 컨테이너 명령어 실행하기 (exec) : 실행중인 컨테이너에 들어가거나 컨테이너의 파일을 실행하는 경우.

```bash
docker exec [OPTIONS] CONTAINER COMMAND [ARG...]

# 새로운 터미널에서 실행되고 있는 컨테이너 실행
docker exec CONTAINER /bin/bash
```

- 컨테이너 실행 (run)

```bash
docker run [OPTIONS] IMAGE [COMMAND] [ARG...]
```

> run은 새로 컨테이너를 만들어서 실행하고 exec은 실행중인 컨테이너에 명령어로 실행한다.

## run 명령어

- `-d` : `-d` 옵션을 사용하면 컨테이너가 detached 모드(백그라운드)에서 실행되며, 실행 결과로 컨테이너 ID만을 출력한다.

- `-it` : `-i` 옵션과 `-t` 옵션은 같이 사용한 경우로, 이 두 옵션은 컨테이너를 종료하지 않은채로, 터미널의 입력을 계속해서 컨테이너로 전달하기 위해서 사용한다. 따라서, 컨테이너의 쉘(shell)이나 CLI 도구를 사용할 때 유용하게 사용된다.

- `--name` : 컨테이너에 이름을 부여하는 옵션으로 해당 이름으로 컨테이너를 식별할 수 있다.

- `-e`, `--env` : Docker 컨테이너의 환경변수를 설정하며, `-e` 옵션은 Dockerfile의 ENV 설정도 덮어쓴다.

- `--ip` : Docker container에 특정 IP를 할당해야 하는 경우 --ip 옵션으로 IP를 할당할 수 있다.

- `-p` : 호스트와 컨테이너 간의 포트(port) 배포(publish)/바인드(bind)를 위해서 사용되어 호스트(host) 컴퓨터에서 컨테이너에서 리스닝하고 있는 포트로 접속할 수 있도록 설정(포트포워딩)해준다.

- `--entrypoint` : Dockerfile의 ENTRYPOINT 설정을 덮어쓰기 위해서 사용

- `--rm` : 컨테이너를 일회성으로 실행할 때 사용되며, 컨테이너가 종료될 때 컨테이너와 관련된 리소스(파일 시스템, 볼륨)까지 깨끗이 제거해준다.

- `--network` : 컨테이너가 사용할 네트워크 선택

# 4. 기본 이미지 다운 및 실행

도커는 이미지를 만들기 위해 컨테이너의 상태를 그대로 이미지로 저장하는 방법을 사용한다.

어떤 애플리케이션을 이미지로 만든다면 리눅스만 설치된 컨테이너에 애플리케이션을 설치하고 그 상태를 그대로 이미지로 저장합니다.

- [우분투 이미지 다운로드](https://hub.docker.com/_/ubuntu)

```bash
docker pull ubuntu:20.04
```

- [ROS 이미지 다운로드](https://registry.hub.docker.com/_/ros/)

```bash
docker pull ros:foxy-ros-core
```

# 참고문헌

- [Docker](https://www.docker.com/)
- [Docker overview](https://docs.docker.com/get-started/overview/)
- [Docker redhat](https://www.redhat.com/ko/topics/containers/what-is-docker)
- [Docker amazon](https://aws.amazon.com/ko/docker/)
- [Docker ROS](https://hub.docker.com/_/ros)
- [ROS Wiki - Docker](http://wiki.ros.org/docker/Tutorials/Docker)
- [SLAMOps를 위한 첫걸음 - Docker + CI](https://www.cv-learn.com/20210808-docker-for-slam/)
- [[Docker] 개념 정리 및 사용방법까지](https://cultivo-hy.github.io/docker/image/usage/2019/03/14/Docker%EC%A0%95%EB%A6%AC/)
- [Docker 개념과 명령어 사용 방법 및 예제](https://kibua20.tistory.com/135)
- [Use the Docker command line](https://docs.docker.com/engine/reference/commandline/cli/)
- [docker run 커맨드 사용법](https://www.daleseo.com/docker-run/)
