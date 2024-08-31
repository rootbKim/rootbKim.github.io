---
layout: archive
title: PostgreSQL
tags: [DB]
category: "Dev"
---

많이 사용되는 DBMS 중 하나인 PostgreSQL에 대해서 알아보고, MySQL과의 차이점 및 PostgreSQL의 장점과 리눅스에서의 설치 및 테스트 방법을 정리한다.

# 1. PostgreSQL이란?

[PostgreSQL](http://www.postgresql.org/)은 북미와 일본에서는 높은 인지도와 많은 인기를 얻고 있는 `RDBMS`다. 국내에서는 아직 잘 사용하지 않고 있지만, 기능과 성능면에서 매우 훌륭한 RDBMS이기 때문에 PostgreSQL가 어떠한 데이터베이스인지 시간을 들여 알아볼 필요는 있다.

`PostgreSQL`(포스트-그레스-큐엘 [Post-Gres-Q-L]로 발음)은 `객체-관계형 데이터베이스 시스템(ORDBMS)`으로, 엔터프라이즈급 DBMS의 기능과 차세대 DBMS에서나 볼 수 있을 법한 많은 기능을 제공하는 `오픈소스` DBMS다. 실제 기능적인 면에서는 `Oracle`과 유사한 것이 많아, Oracle 사용자들이 가장 쉽게 적응할 수 있는 오픈소스 DBMS가 PostgreSQL이라는 세간의 평 또한 많다.

# 2. MySQL vs. PostgreSQL

DBMS를 선택할 때, 어떤 DBMS를 사용할 것인지에 대한 고민은 필요하다. DBMS마다 가지는 장점과 단점이 있을 것이고, 이러한 장단점을 고려해, 프로젝트의 목적에 맞게 효과적인 DBMS를 선택하는 것이 중요할 것이다. 여기서는 해당 포스팅에서 다루고자 하는 PostgreSQL과 널리 사용되고 있는 MySQL의 장점을 비교해보고자 한다.

### MySQL의 장점

1. 고도의 유연성 및 확장성: MySQL을 사용하면 스토리지 엔진에 대한 선택의 폭이 커진다. 따라서 다양한 테이블 유형의 데이터를 유연하게 통합할 수 있다. MySQL 8.0은 다음과 같은 스토리지 엔진을 지원한다다.
  - InnoDB, MyISAM, Memory, CSV, Archive, Blackhole, NDB/NDBCLUSTER,  Merge, Federated, Example
2. 속도 및 안정성에 집중: MySQL은 특정 SQL 기능을 포함하지 않음으로써 속도와 안전성에 지속적으로 우선순위를 둔다. MySQL의 속도는 고도의 동시 읽기 전용 기능에서 특히 두드러진다. 이러한 이유로 특정 비즈니스 인텔리전스용으로는 탁월한 선택이다. 그렇지만 **부하가 많은 상태에서 복잡한 쿼리를 대량으로 실행해야 한다면 PostgreSQL이 더 나은 선택이 될 수 있다**.
3. 서버 최적화를 위한 옵션: MySQL은 sort_buffer_size, read_buffer_size, max_allowed_packet 등 변수를 조정하여 MySQL 서버를 수정 및 최적화하는 옵션을 많이 제공한다.
4. 사용의 용이성 및 대중성: MySQL이 널리 사용된다는 것은 광범위한 MySQL 경험이 있는 데이터베이스 관리자를 쉽게 찾을 수 있다는 것을 의미한다. 사용자들에 따르면, MySQL이 설치가 용이하고 다른 DBMS 솔루션보다 미세 조정의 필요성이 크지 않다. 이 튜토리얼에서는 초보자가 처음으로 MySQL 데이터베이스를 설치하는 것이 얼마나 쉬운지를 보여 준다. 또한, 다양한 프론트엔드(예: Adminer, MySQL Workbench, HeidiSQL, dbForge Studio)가 보다 사용자 친화적인 경험을 제공하는 그래픽 인터페이스를 MySQL에 추가한다.
5. 클라우드 지원 DBMS: MySQL은 클라우드를 지원한다. 여러 클라우드 플랫폼에서 MySQL 데이터베이스를 유료로 설치하고 유지 관리해주는 MySQL 기능을 제공한다.
6. InnoDB 엔진에서 다중 버전 동시성 제어(MVCC) 및 ACID 규정 준수 제공: 현재 MySQL 버전의 기본 엔진은 InnoDB으로, MVCC와 ACID 규정 준수가 추가되었다. 그러나 MySQL의 InnoDB는 MyISAM 테이블 형식 때문에 테이블 오염 관련 문제가 계속해서 발생할 수 있다. MySQL에 따르면, "MyISAM 테이블 형식은 매우 신뢰할 만하지만(SQL 문에 의한 테이블의 모든 변경 사항은 해당 SQL 문이 반환되기 전에 작성됨) 테이블의 오염 가능성은 여전히 존재"한다고 한다. 그리고 다른 엔진을 선택하면 MVCC와 ACID 규정을 준수하지 못하는 결과가 초래될 수 있다.

### PostgreSQL의 장점

1. 단순히 RDBMS가 아닌 ORDBMS: PostgreSQL은 개체 관계형 프로그래밍 언어(ORDBMS)로서 개체 지향 프로그래밍과 관계 지향/절차 지향 프로그래밍 사이를 연결하는 역할을 한다(C++와 유사). 따라서 개체 및 테이블 상속을 정의할 수 있으므로 데이터 구조가 더 복잡해진다. ORDBMS는 엄격한 관계형 모델에 맞지 않는 데이터를 처리할 때 탁월하다.
2. 복잡한 쿼리에 탁월: 유효성 검사가 필요한 데이터를 사용하면서 복잡한 읽기-쓰기 작업을 수행해야 하는 경우 PostgreSQL이 유리하다. 그러나 ORDBMS는 읽기 전용 작업을 처리할 때 속도가 느려질 수 있다(**이 경우 MySQL이 탁월**).
3. NoSQL 및 다양한 데이터 형식 지원: NoSQL 기능을 고려하는 경우 PostgreSQL을 많이 선택한다. PostgreSQL은 JSON, hstore, XML 등 매우 다양한 데이터 형식을 기본적으로 지원한다. 사용자는 본래의 데이터 형식을 정의하는 것은 물론, 사용자 지정 함수도 설정할 수 있다.
4. 초대형 데이터베이스 관리용으로 설계: PostgreSQL은 데이터베이스의 크기에 제한을 두지 않는다.
5. 다중 버전 동시성 제어(MVCC): MVCC를 통해 데이터를 읽는 사람과 작성하는 사람이 서로 통신하여 PostgreSQL 데이터베이스를 동시에 관리할 수 있다. 따라서 데이터와 통신해야 할 때마다 읽기-쓰기 잠금을 할 필요가 없으므로 효율성이 향상된다. MVCC는 이것을 "스냅샷 격리"(Oracle에서 지칭하는 방법)를 통해 구현한다. 스냅샷은 특정 순간의 데이터 상태를 나타낸다.
6. ACID 준수: PostgreSQL은 데이터 오염을 방지하고 트랜잭션 수준에서 데이터 무결성을 보존한다.
7. 라이선스에 대한 비용문제가 전혀 없음: **BSD 라이선스**이며, 라이선스의 가장 큰 특징은 소스를 변경하고 그 소스를 숨긴 채 재배포 해도 법적으로 문제가 없다는 점이다. PostgreSQL을 사용하는 기업들의 가장 큰 이유이지 않을까 생각한다.
8. 오래된 오픈소스의 안정성: 매우 가볍게 돌아가는 데이터베이스지만, 대용량 데이터의 처리에도 큰 문제점이 발견되지 않았고, 표준SQL 잘 따르고있다.
9. Oracle에 버금가는 통계함수의 지원
10. 여전히 발전중인 데이터베이스
11. 장점이자 단점인 독창적인 자료형 및 문법: 표준SQL 만으로 PostgreSQL를 사용해도 좋다. 하지만 독창적인 자료형,문법, 함수들을 익힌다면 더욱 효과적이고 파워풀하게 데이터베이스를 활용할 수 있다. (예를 들자면 ARRAY, JSON, RANGE등의 자료형, ILIKE 등의 문법은 개발자들에게 큰 편의성과 효율성을 제공한다)

# 3. PostgreSQL 설치

우분투 환경에서 PostgreSQL의 설치 방법에 대해서 알아보고, 계정 및 DB 생성과 연결을 통한 TEST 방법에 대해서 알아본다.

### 설치

```bash
sudo apt install postgresql
```

### 설치 확인

```bash
sudo systemctl is-active postgresql
sudo systemctl is-enable postgresql
sudo systemctl status postgresql
```

### 클라이언트 연결 준비상태 확인

```bash
sudo pg_isready
```

### postgresql 접속

```bash
# postgres 시스템 사용자 계정으로 전환
sudo su - postgres

# postgresql 접속
psql
```

### 사용자 계정 생성 및 db 생성 / 연결

```bash
postgres=# CREATE USER [USERID] WITH PASSWORD '[USERPW]';
postgres=# CREATE DATABASE [DBNAME];
postgres=# GRANT ALL PRIVILEGES ON DATABASE [DBNAME] to [USERID];
```

다음은 TEST 용 user id와 password 및 db를 생성한 것이다.
- USERID : test
- USERPW : sdf
- DBNAME : testdb

<img src="/assets/img/posts/230204_postgresql.png">

# 참고문헌

- [한눈에 살펴보는 PostgreSQL](https://d2.naver.com/helloworld/227936)
- [PostgreSQL과 MySQL 비교: 주요 차이점](https://www.integrate.io/ko/blog/postgresql-vs-mysql-the-critical-differences-ko/)
- [PostgreSQL을 선택한 이유](https://codecamp.tistory.com/2)
- [[DB] PostgreSQL이란? 및 설치 방법](https://learning-e.tistory.com/25)
- [Ubuntu 20.04에서 PostgreSQL 및 pgAdmin4를 설치하는 방법](https://ko.linux-console.net/?p=748#gsc.tab=0)
- [Ubuntu에 PostgreSQL 설치하고 기본 명령 살펴보기](https://dejavuqa.tistory.com/16)