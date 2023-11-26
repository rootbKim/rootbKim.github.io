---
layout: note_page
title: Docker etc.
tags: [Docker]
category: "Dev"
---

Docker 유용한 기능에 대해서 정리한다.

## 1. Docker Image Export/Import to FILE

Export: 도커 이미지를 파일로 추출

```bash
docker save [이미지명] > [추출파일명].tar
```

Import: 추출된 파일을 도커 이미지로 로드
```bash
docker load < [추출파일명].tar
```

## 2. Docker cp

`scp` 명령처럼 `docker cp` 명령어를 이용하여 host에서 docker로, docker에서 host로 파일 또는 디렉토리를 전송할 수 있다.

```bash
# Host -> Docker
docker cp [전송할 데이터 path] [컨테이너 ID]:[컨테이너 내 전송받을 path]

# Docker -> Host
docker cp [컨테이너 ID]:[컨테이너 내 전송할 데이터 path] [전송받을 path]
```

## 3. Docker ssh

도커를 실행 시 `22`번 포트를 바인딩 시켜줘야 한다. 여기서는 host의 `5222`` 포트를 컨테이너의 `22`` 포트와 바인딩한다.

```bash
docker run -it -p 5222:22 [도커 이미지]
```

실행되고 있는 도커 컨테이너에 ssh 접속하기 위한 방법으로, docker 컨테이너 내에서 다음 패키지를 설치한다.

```bash
apt-get install passwd
apt-get install vim
apt-get install net-tools
apt-get install openssh-server
```

`/etc/ssh/sshd_config` 내의 `PermitRootLogin`, `PasswordAuthentication` 필드 수정

```text
vi /etc/ssh/sshd_config

...
PermitRootLogin yes

...
PasswordAuthentication yes
```

root 계정 암호 설정

```bash
passwd root
```

ssh 실행

```bash
service ssh start
service ssh status # ssh 서비스가 실행 중: sshd is running
```

host에서 ssh 명령으로 접속할 수 있다.

```bash
ssh root@127.0.0.1 -p 5222
```

## 4. attach와 exec

여러 개의 터미널에서 하나의 컨테이너에 접속하기 위해 `attach`를 하는 방법과 `exec`을 이용하는 방법이 있다.

`attach`의 경우 실행되고 있는 컨테이너에 붙는 것으로, 컨테이너와 동일한 화면을 보게 된다. 즉, 독립적인 명령을 수행할 수 없다.

```bash
docker attach [컨테이너 이름]
```

`exec`의 경우 `attach`와 다르게 외부에서 진입하는 명령으로, 새로운 터미널로 접속이 되어 독립적인 명령을 수행할 수 있다.

```bash
docker exec -it [컨테이너 이름] /bin/bash
```

## 5. Docker 간 통신(TODO)

- [[Network] 컨테이너간 네트워크 (포트 관련)](https://jm4488.tistory.com/18)
- [Docker Network 구조(3) - container 외부 통신 구조](https://bluese05.tistory.com/53)
- [도커 컨테이너 간 통신](https://velog.io/@760kry/%EB%8F%84%EC%BB%A4-%EC%BB%A8%ED%85%8C%EC%9D%B4%EB%84%88-%EA%B0%84-%ED%86%B5%EC%8B%A0)
- [06. 도커 네트워크 (Docker Network)](https://captcha.tistory.com/70)
- [Docker 네트워크 사용법](https://www.daleseo.com/docker-networks/)
- [Docker Container Network 중 Bridge에 대한 설명 정리](https://jangseongwoo.github.io/docker/docker_container_network/)

## 6. Docker GUI(TODO)

- [Docker에서 GUI 환경 구축](https://blog.naver.com/PostView.naver?blogId=qbxlvnf11&logNo=222440984918&categoryNo=0&parentCategoryNo=0&viewDate=&currentPage=1&postListTopCurrentPage=1&from=postView)
- [[Docker] Docker를 이용하여 ROS 실행 및 GUI 보는 방법](https://jstar0525.tistory.com/333)
- [4일차 - Ubuntu 18.04.2 LTS에 Docker 이용하여 ROS 설치하기](https://roomedia.tistory.com/entry/4%EC%9D%BC%EC%B0%A8-Ubuntu-18042-LTS%EC%97%90-Docker-%EC%9D%B4%EC%9A%A9%ED%95%98%EC%97%AC-ROS-%EC%84%A4%EC%B9%98%ED%95%98%EA%B8%B0)

## 참고문헌

- [[Docker] Docker 이미지를 파일로 import/export 하기](https://engineer-mole.tistory.com/257)
- [[docker] container host간의 데이터 전송](https://velog.io/@starbirdnara/docker-container-host%EA%B0%84%EC%9D%98-%EB%8D%B0%EC%9D%B4%ED%84%B0-%EC%A0%84%EC%86%A1)
- [[Docker] 도커에서 올린 ubuntu 컨테이너에 ssh로 접근하기 (feat.PuTTY)](https://jong-bae.tistory.com/14)
- [[Docker] SSH로 Docker Contrainer에 접속하는 방법](https://prup.tistory.com/55)
- [docker의 ubuntu container에 ssh로 접속하기](https://chanhy63.tistory.com/11)
- [[Docker] 터미널 여러개 접속하기](https://velog.io/@leeyejin1231/Docker-%ED%84%B0%EB%AF%B8%EB%84%90-%EC%97%AC%EB%9F%AC%EA%B0%9C-%EC%A0%91%EC%86%8D%ED%95%98%EA%B8%B0)
