---
layout: note_page
title: Message Queue
tags: [Communication, MQ]
category: "Dev"
---

Message Queue의 전체적인 개념과 Message Queue의 특징 및 종류를 정리한다.

# 1. Message Queue 개념

메시지 큐(Message Queue)는 프로세스 또는 프로그램 간에 데이터를 교환할 때 사용하는 통신 방법 중에 하나로, 메시지 지향 미들웨어(Message Oriented Middleware: MOM)를 구현한 시스템을 의미한다. 메시지 지향 미들웨어란 비동기 메시지를 사용하는 응용 프로그램들 사이에서 데이터를 송수신하는 것을 의미한다. 여기서 메시지란 요청, 응답, 오류 메시지 혹은 단순한 정보 등의 작은 데이터가 될 수 있다.

<img src="/assets/img/posts/230612_message_queue.png">

메시지 큐는 메시지를 임시로 저장하는 간단한 버퍼라고 생각하면 된다. 메시지를 전송 및 수신하기 위해 중간에 메시지 큐를 두는 것이다.

메시지 전송 시 생산자(Producer)로 취급되는 컴포넌트가 메시지를 메시지 큐에 추가한다. 해당 메시지는 소비자(Consumer)로 취급되는 또 다른 컴포넌트가 메시지를 검색하고 이를 사용해 어떤 작업을 수행할 때까지 메시지 큐에 저장된다. 각 메시지는 하나의 소비자에 의해 한 번만 처리될 수 있는데, 이러한 이유로 메시지 큐를 이용하는 방식을 일대일 통신이라고 부른다.

## 1.1 Message Queue의 특징

### 비동기(Asynchronous)
메시지 큐는 생산된 메시지의 저장, 전송에 대해 동기화 처리를 진행하지 않고, 큐에 넣어 두기 때문에 나중에 처리할 수 있다. 여기서, 기존 동기화 방식은 많은 메시지(데이터)가 전송될 경우 병목이 생길 수 있고, 뒤에 들어오는 요청에 대한 응답이 지연될 것이다.

### 낮은 결합도(Decoupling)
생산자 서비스와 소비자 서비스가 독립적으로 행동하게 됨으로써 서비스 간 결합도가 낮아진다.

### 확장성(Scalable)
생산자 서비스 혹은 소비자 서비스를 원하는 대로 확장할 수 있기 때문에 확장성이 좋다.

### 탄력성(Resilience)
소비자 서비스가 다운되더라도 어플리케이션이 중단되는 것은 아니다. 메시지는 메시지 큐에 남아 있다. 소비자 서비스가 다시 시작될 때마다 추가 설정이나 작업을 수행하지 않고도 메시지 처리를 시작할 수 있다.

### 보장성(Guarantees)
메시지 큐는 큐에 보관되는 모든 메시지가 결국 소비자 서비스에게 전달된다는 일반적인 보장을 제공한다.

## 1.2 Message Queue의 사용
위와 같은 Message Queue의 특징 때문에 다음과 같은 상황이 있을 수 있다.

* 메시지 큐는 소비자(Consumer)가 실제로 메시지를 어느 시점에 가져가서 처리하는 지는 보장하지 않는다.
* 언젠가는 큐에 넣어둔 메시지가 소비되어 처리될 것이라고 믿는 것이다.
* 사용자가 많아지거나 데이터가 많아지면 응답 지연으로 서비스가 정상적으로 작동하지 못하는 상황이 올 수 있다.
* 비동기적 특성 때문에 메시지 큐는 실패하면 치명적인 핵심 작업보다는 어플리케이션의 부가적인 기능에 사용하는 것이 적합하다.

이러한 Message Queue의 성질로 다음과 같은 상황을 고려할 수 있다.
* 간단한 서버 구조에서는 굳이 사용자의 요청과 응답에 시간이 소요되는 메시지큐를 사용할 이유는 없는다.
* 대용량 데이터를 처리하기 위한 배치 작업, 채팅 서비스, 비동기 데이터를 처리할 때 사용한다.
    * 많은 양의 데이터를 분산 처리하지 않고, 메시지 브로커를 통하여 한 곳에서 데이터를 처리하고 필요한 프로그램에 작업을 분산시켜야 하는 경우에 사용할 수 있다.
    * 다양한 애플리케이션과의 비동기 통신
    * 많은 양의 프로세스들을 처리
    * 이메일 발송 및 문서 업로드

## 1.3 Message Queue 프로토콜
Message Queue의 Producer와 Consumer의 정보 교환에는 다양한 방법론들과 프로토콜들이 존재하는데, 대표적으로 MQTT, AMQP, JMS(JMS는 프로토콜은 아니고 API)와 같은 것들이 존재한다.

* MQTT: MQTT(Message Queue Telemetry Transport)는 Publish-Subscribe 구조의 메시지 송수신 프로토콜로, HTTP 프로토콜에 비해 제한된 통신 환경과 낮은 전력으로 사용할수 있기에 주로 IoT 부야와 메신져 분야에서 사용됨.
* AMQP: AMQP(Advanced Message Queing Protocol)는 MOM을 위한 Message Queue의 오픈소스에 기반한 표준 프로토콜을 의미
* JMS: JMS는 MOM를 자바에서 지원하는 표준 API로, 다른 자바 애플리케이션들끼리 통신은 가능하지만 다른 MOM과는 통신 불가하다.

# 2. Message Queue 종류
## 2.1 RabbitMQ
RabbitMQ는 AMQP(Advanced Message Queuing Protocol)를 구현한 오픈소스 메시지 브로커이다.
* 신뢰성, 안정성과 성능을 충족할 수 있도록 다양한 기능 제공
* 유연한 라우팅: Message Queue가 도착하기 전에 라우팅 되며 플러그인을 통해 더 복잡한 라우팅도 가능
* 클러스터링: 로컬 네트워크에 있는 여려 RabbitMQ 서버를 논리적으로 클러스터링 할 수 있고 논리적인 브로커도 가능 함.
* 오픈 소스로 상업적 지원이 가능
* AMQP는 메세지 전달을 아래 3가지 방식 중 하나를 보장한다.
    * At-Most-Once: 각 메시지는 한번만 전달되거나 전달되지 않음
    * At-Least-Once: 각 메시지는 최소 한번 이상 전달됨을 보장
    * Exactly-Once: 각 메시지는 딱 한번만 전달됨

## 2.2 Mosquitto
* MQTT 프로토콜 이용
* C 기반
* 가볍고, 기본에 충실

## 2.3 ActiveMQ
* JMS API를 사용한다.
* 다양한 언어 환경의 클라이언트와 프로토콜 지원
* Spring 지원으로 ActiveMQ는 Spring 애플리케이션에 쉽게 임베딩 + XML 설정이 쉬움
* REST API를 통해 웹 기반 메시징 지원
* RabbitMQ와는 다르게 모니터링 도구가 부실함
* Dead letter queue 지원

## 2.4 ZeroMQ
* 분산/동시성 애플리케이션에 사용되도록 개발됨
* 메세지 지향 미들웨어와 달리 메세지 브로커 없이 동작 가능 (크로스 플랫폼으로 동작 가능)
* 다양한 기능들을 제공하여 확장성이 뛰어남
* 복잡한 프로토콜이 없기 때문에 대부분의 AMQP들 빠르다.
* 비동기 send 호출을 부르기만 하면, 메시지를 별도의 스레드의 큐에 넣고 필요한 모든 일을 처리한다.
* 다음과 같은 패턴을 제공함
    * Request-Reply : 클라이언트와 서비스의 집합을 연결하는 패턴
    * Publish-subscribe : publisher와 subscriber 집합을 연결하는 패턴
    * Pipeline : Push/Pull 소켓 쌍으로 단방향 통신에 이용하는 패턴

## 2.5 Kafka
* 대용량 실시간 로그 처리에 특화되어 있다.
* AMQP 프로토콜이나 JSM API를 사용하지 않고 단순한 메세지 헤더를 지닌 TCP 기반 프로토콜을 사용하므로서 오버헤드가 비교적 작다.
* 노드 장애에 대한 대응성을 가지고 있다.
* 프로듀서는 각 메세지를 배치로 브로커에 전달하며 TCP/IP 라운드 트립을 줄였다.
* 메세지를 기본적으로 파일 시스템에 저장하여 별도의 설정을 하지 않아도 오류 발생시 오류 지점부터 복구가 가능 (기존 메세징 시스템은 메시지를 메모리에 저장)
* 메시지를 파일 시스템에 저장하기 떄문에 메세지가 많이 쌓여도 기존 메세징 시스템에 비해 성능이 크게 감소하지 않는다.
* window 단위의 데이터를 넣고 꺼낼수 있다.

# 참고문헌

- [[오픈소스] 메시지큐(Message Queue) 알아보기](https://12bme.tistory.com/176) 
- [[IT정보] 메시지 큐(Message Queue, MQ) 개념](https://blog.naver.com/seek316/222117711303)
- [메시지 큐에 대해 알아보자!](https://tecoble.techcourse.co.kr/post/2021-09-19-message-queue/) 
- [[개념원리]Message Queue](https://velog.io/@power0080/Message-Queue-%EA%B0%9C%EB%85%90-%EC%A0%95%EB%A6%AC)
- [Message Queue 란? MQ 종류](https://goyunji.tistory.com/125)
- [MQTT AMQP RabbitMQ](https://hyunalee.tistory.com/39)
- [Message Broker - 왜 사용하는 것일까?](https://binux.tistory.com/74)