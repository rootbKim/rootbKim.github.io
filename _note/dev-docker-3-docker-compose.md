---
layout: note_page
title: Docker Compose, Volume, Network
tags: [Docker]
category: "Dev"
---

Docker Compose를 이용한 여러 개의 컨테이너를 한 번에 관리하는 방법에 대해서 정리한다.

## 1. Docker Compose

도커 컴포즈는 여러 개의 컨테이너의 옵션과 환경을 정의한 파일을 읽어 컨테이너를 순차적으로 생성하는 방식으로 동작하게 한다. 도커 컴포즈의 설정 파일은 도커 엔진의 run 명령어의 옵션을 그대로 사용할 수 있으며, 각 컨테이너의 의존성, 네트워크, 볼륨 등을 함께 정의할 수 있다.

### 1.1 Install Docker Compose

[Docker compose 설치 홈페이지](https://docs.docker.com/compose/install/standalone/)에서 curl을 이용한 설치 명령어를 이용하여 설치한다. 아래의 명령어를 그대로 사용해도 되지만, 최신 버전이 아닐 수 있으므로, 홈페이지에서 명령어를 가져오거나, 최신 버전을 [git release 페이지](https://github.com/docker/compose/releases)에서 확인해볼 수 있다.

```bash
sudo curl -SL https://github.com/docker/compose/releases/download/v2.23.3/docker-compose-linux-x86_64 -o /usr/local/bin/docker-compose
```

다운로드한 docker compose에 실행권한을 부여하면 docker compose를 사용할 수 있다.

```bash
sudo chmod +x /usr/local/bin/docker-compose
```

### 1.2 docker-compose.yaml

도커 컴포즈는 `docker-compose.yaml` 파일을 읽어서 컨테이너를 생성한다. 다음은 `postgresql`, `redis`, `rabbitmq` 컨테이너를 생성하는 `docker-compose.yaml` 파일이다.

```yaml
version: '3'
services:
  postgres:
    image: postgres:15.4
    container_name: tms-postgres
    ports:
      - '5432:5432'
    volumes:
      - ./volumes/postgres:/var/lib/postgresql/data
    environment:
      - POSTGRES_USER=postgres
      - POSTGRES_PASSWORD=postgres
      - POSTGRES_DB=postgres
  redis:
    image: redis:7.2.0
    container_name: tms-redis
    ports:
      - '6379:6379'
    volumes:
      - ./volumes/redis:/data
  rabbitmq:
    image: rabbitmq:3.12
    container_name: tms-rabbitmq
    ports:
      - '5671:5671'
      - '5672:5672'
      - '15672:15672'
    volumes:
      - ./volumes/rabbitmq:/var/lib/rabbitmq
```

도커 컴포즈는 위와 같은 `docker-compose.yaml` 파일을 읽어 로컬의 도커 엔진에게 각각의 컨테이너 생성을 요청한다. 이러한 방법은 docker run을 이용해 각각의 컨테이너를 실행하는 방법을 하나의 yaml 파일에 정의함으로써 한번에 컨테이너를 생성할 수 있음을 보여준다.

### 1.3 docker compose의 실행 및 종료

도커 컴포즈를 실행 및 종료하기 위해서는 다음과 같은 명령을 사용할 수 있다.

```bash
docker-compose up                   # 현재 위치의 docker-compose.yaml 파일을 읽어서 컴포즈 실행
docker-compose up -d                # 백그라운드 실행
docker-compose -p my_project up -d  # my_project 이름으로 변경하여 컴포즈 실행

docker-compose down                 # 실행 중인 컴포즈 종료
docker-compose down -v              # 실행 중인 컴포즈 종료 및 볼륨 제거

docker-compose ps                   # 컴포즈 목록 확인
```

> 도커 컴포즈를 실행했을 때 실행에 필요한 이미지가 없다면 이미지를 자동으로 pull 하여 설치하여 실행하며, 만약 로컬에 이미지를 갖고 있다면, 로컬에 있는 이미지를 실행한다.

### 1.4 docker-compose.yaml 옵션

도커 컴포즈 yaml 파일에 정의되는 각 테그에 대해서 간략하게 정리한다.

* `version`: yaml 파일 포맷의 버전
* `services`: 생성될 컨테이너를 묶어놓은 단위
* `image`: 컨테이너를 생성할 때 사용할 이미지 이름. 로컬에 해당하느 이미지가 없는 경우 도커 허브에서 내려받음
* `links`: `docker run` 명령어의 `--link` 옵션. 다른 서비스에서 서비스명만으로 접근 가능할 수 있도록 설정
* `environment`: `docker run` 명령어의 `--env, -e` 옵션. 컨테이너 내부의 환경변수 설정
* `command`: 컨테이너가 실행될 때 수행할 명령어를 설정.
* `depends_on`: 특정 컨테이너에 대한 의존 관계를 나타냄. 이 항목에 명시된 컨테이너가 먼저 실행되고, 해당 컨테이너가 그 이후에 실행됨.
* `ports`: `docker run` 명령어의 `-p` 옵션. 포트포워딩

## 2. 도커 볼륨

도커 컨테이너는 일반적으로 컨테이너가 실행되고 종료되면 안에서 생성된 데이터는 사라지게 된다. 도커 컨테이너에서 생성된 데이터를 보존하기 위해서 사용되는 개념이 도커 마운트와 도커 볼륨이다.

여기서의 볼륨의 개념이 `docker-compose.yaml`의 `volumes`에 해당하는 내용이다. 즉, 해당 볼륨 설정으로 데이터를 보관하게 할 수 있다. `postgresql`의 경우 `/var/lib/postgresql/data`에 데이터를 저장하는데, 컨테이너 내의 이 폴더를 호스트의 `volumes/postgres`에 보관하게 함으로써 컨테이너가 삭제되어도 데이터를 유지하게 할 수 있다.

### 도커 마운트

도커 마운트는 호스트 상의 경로와 컨테이너를 연결하는 개념으로 다음 링크에 예시가 정리되어 있다: [[Infra] 도커(docker)(9) 도커 PostgreSQL 컨테이너 배포하기](https://losskatsu.github.io/it-infra/docker09/#25-%EB%8F%84%EC%BB%A4-%EB%A7%88%EC%9A%B4%ED%8A%B8mount---%ED%98%B8%EC%8A%A4%ED%8A%B8-%EA%B2%BD%EB%A1%9C%EC%99%80-%EC%BB%A8%ED%85%8C%EC%9D%B4%EB%84%88-%EC%97%B0%EA%B2%B0)

### 도커 볼륨

도커 볼륨은 호스트 시스템과 도커 컨테이너 간의 데이터를 공유하고 보관하기 위한 기능으로, 다음 링크에서 예시를 확인해볼 수 있다.: [[Infra] 도커(docker)(9) 도커 PostgreSQL 컨테이너 배포하기](https://losskatsu.github.io/it-infra/docker09/#26-%EB%8F%84%EC%BB%A4-%EB%B3%BC%EB%A5%A8volume), [Docker 컨테이너에 데이터 저장 (볼륨/바인드 마운트)](https://www.daleseo.com/docker-volumes-bind-mounts/)

## 3. 도커 포트포워딩

도커는 `eth0` 네트워크 인터페이스에 내부망 IP 주소로 Gateway가 `172.17.0.1` (netmask 255.255.0.0)으로 자동 설정된다. 이 네트워크는 `docker0` 라는 네트워크 인터페이스에 연결이 되는데, 도커 컨테이너가 생성 될 때 이 docker0에 연결되며 ip는 컨테이너 실행 순서에 따라 순차적으로 할당된다

이 도커 컨테이너는 생성 시 내부망 IP를 사용하므로 호스트에서 컨테이너로의 접속이 불가능한데, 이러한 문제를 해결하게 위해 포트포워딩을 사용한다. `docker run`으로 컨테이너를 실행할 때 `-p` 옵션으로 설정할 수 있으며, 그 방법은 다음과 같다.

```bash
docker run -p <host port>:<container port>/<protocol> [IMAGE NAME] [OTHER OPTIONS ...]
```

여기서 `<host port>` 를 `<container port>`를 연결해주며, `<protocol>`은 프로토콜 유형으로 `udp`, `tcp`, `stcp` 등의 옵션을 설정할 수 있다.

`docker-compose.yaml`의 `ports`의 항목이 도커 포트포워딩을 해주는 부분인데, 프로토콜은 명시하지 않으면 자동으로 `tcp`로 설정된다. `postgresql`의 경우 `5432` 포트를 사용하는데, 이를 호스트의 `5432` 포트로 포트포워딩을 함으로써 호스트에서 `5432` 포트로 접근하면 도커 컨테이너의 DB에 접근할 수 있다.

> 만약에 호스트에서 `5432` 포트를 사용 중이라면, 이미 포트를 사용 중이므로 실행할 수 없게 된다.

## 참고문헌

- [Docker compose 설치 홈페이지](https://docs.docker.com/compose/install/standalone/)
- [Docker compose git release 페이지](https://github.com/docker/compose/releases)
- [[Docker] 도커 컴포즈(Docker compose) - 개념 정리 및 사용법](https://seosh817.tistory.com/387)
- [[Infra] 도커(docker)(9) 도커 PostgreSQL 컨테이너 배포하기](https://losskatsu.github.io/it-infra/docker09/#)
- [Docker 컨테이너에 데이터 저장 (볼륨/바인드 마운트)](https://www.daleseo.com/docker-volumes-bind-mounts/)
- [도커(Docker) : 포트 포워딩 설정(포트 맵핑)하기](https://tttsss77.tistory.com/155)
- [Docker Container Network 중 Bridge에 대한 설명 정리](https://jangseongwoo.github.io/docker/docker_container_network/)
- [[web] Docker 를 이용한 DB 사용법](https://velog.io/@wonizizi99/DB-Doker-%EB%A5%BC-%EC%9D%B4%EC%9A%A9%ED%95%9C-DB-%EC%82%AC%EC%9A%A9%EB%B2%95)
- [PostgreSQL, MySQL Docker 데이터베이스 초기화 하기](https://judo0179.tistory.com/69)
- [Creating and filling a Postgres DB with Docker compose](https://levelup.gitconnected.com/creating-and-filling-a-postgres-db-with-docker-compose-e1607f6f882f)
