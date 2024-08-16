---
layout: note_page
title: Github SSH 등록하기
tags: [Git]
category: "Dev"
---

원격저장소인 GitHub과 통신하기 위한 SSH(Secure Shell) 프로토콜에 대해서 알아보고, SSH Key의 생성 및 등록 방법과 사용 방법에 대해 다룬다. 또한 여러 개의 GitHub 계정을 사용할 때 여러 개의 SSH Key를 등록하는 방법에 대해서도 정리한다.

# SSH Key

SSH ( Secure Shell Protocol ) 는 **보안되지 않은 네트워크에서 네트워크 서비스를 안전하게 운영하기 위한 암호화 네트워크 프로토콜**이다. 

SSH 애플리케이션은 SSH 클라이언트 인스턴스를 SSH 서버 와 연결하는 클라이언트-서버 아키텍처 를 기반으로 구성되며, SSH는 세 가지 주요 계층 구성 요소로 동작한다. 전송 계층(Transport Layer)은 서버 인증, 기밀성 및 무결성을 제공하고, 사용자 인증 프로토콜(User Authentication Protocol)은 서버에 대한 사용자의 유효성을 검사, 연결 프로토콜(Connection Protocol)은 암호화된 터널을 여러 논리적 통신 채널로 다중화하는 기능을 한다.

<img src="/assets/img/posts/221219_ssh_protocol.png">

SSH 프로토콜은 클라이언트에서 서버에 접속할때 key를 제출하는 방식으로, 비밀번호보다 높은 수준의 보안요건을 필요로 할때 사용된다. SSH key는 공개키와 비공개키로 이루어진다. SSH 키를 생성하면 공개키와 비공개키가 만들어지고, 이 중 공개키를 서버에 제공해야 한다.

클라이언트에서 SSH 접속을 시도하면 클라이언트와 서버는 각각 가지고 있는 키를 가지고 서로의 키를 확인하여 인증 절차를 거친 후, 인증이 완료되면 SSH 세션을 수행한다.

# GitHub 과 SSH Key

다음은 GitHub에서 설명하는 SSH에 대한 설명이다.

> SSH 프로토콜을 사용하여 원격 서버 및 서비스에 연결하고 인증할 수 있다. SSH 키를 사용하면 각 저장소에 대한 접근 시 사용자 이름 및 personal access token를 제공하지 않고도 GitHub에 연결할 수 있다. 또한, SSH 키를 사용하여 커밋에 서명할 수도 있다.

즉, 장치의 SSH Key만 GitHub에 공유된다면, 토큰(token)을 사용하지 않고 등록되어 있는 key로 손쉽게 인증하고 접근할 수 있다는 것이다.

기존에는 GitHub에 있는 레포지토리에 접근하려면 ID/PW를 사용했으나, 어느 시점부터 personal access token을 발행해서 사용하도록 하였다. 이 토큰을 GitHub에 접근할 때마다 입력해야 하니 번거로운 작업이 되지만, SSH Key는 등록만 해놓으면 매우 편리하게 GitHub에 접근을 할 수 있게 된다.

# SSH Key 생성 및 등록 방법

## SSH Key 생성

Linux에서 SSH 키를 등록하기 위해서는 다음의 명령을 사용한다.

```bash
ssh-keygen -t ed25519 -C "your_email@example.com" -f "key_file_name"

# 만약 ed25519가 지원되지 않는다면 rsa 방법을 사용
ssh-keygen -t rsa -b 4096 -C "your_email@example.com"
```

SSH 키를 생성하면 `~/.ssh` 위치에 `key_file_name`과 `key_file_name.pub`이 생성이 된다.
`key_file_name`이 비공개키이고, `key_file_name.pub`이 공개키에 해당한다. 이 때, 보안을 위해서 비공개키는 공개되지 않도록 유의해야 한다.

## SSH Key 등록

SSH 키를 만들었다면, 이제 만들어진 공개키를 GitHub에 등록해야 개인키를 이용해 GitHub에 접속할 수 있다.

우선 `key_file_name.pub`의 내용을 복사한다.

```bash
cat ~/.ssh/"key_file_name.pub"
```

다음으로 GitHub 계정에 접속하여, `Settings`의 `SSH and GPG keys`에 들어가면, 다음과 같이 등록되어 있는 SSH 키 목록과, SSH 키를 새로 등록할 수 있다.

<img src="/assets/img/posts/221219_ssh_settings.png">

`New SSH key`를 눌러, 키의 이름과 위에서 복사한 키를 입력하고 `Add SSH key`를 누르면 키가 등록이 되며, 리스트에도 설정한 이름으로 추가된 것을 볼 수 있다.

# 여러 개의 SSH Key 등록 방법

한 대의 컴퓨터에서 여러 개의 계정을 사용해야 하는 경우, 각 계정마다 SSH 키를 등록해야 한다. 컴퓨터에 SSH 키가 여러 개의 계정으로 여러 개가 존재하는 경우 추가적인 설정이 필요하다.

예를 들어, `key1` 과 `key2` 라는 키를 생성하여, `~/.ssh` 위치에 `key1`, `key1.pub`, `key2`, `key2.pub`이 생성되어 있다고 가정하자.

```bash
ssh-keygen -t ed25519 -C "key1@example.com" -f "key1"
ssh-keygen -t ed25519 -C "key2@example.com" -f "key2"
```

다음으로 각각의 키와 계정을 맵핑시켜줄 수 있는 `config` 파일을 다음과 같이 만든다.

```text
Host github.com-key1
        HostName github.com
        User key1@example.com
        IdentityFile ~/.ssh/key1
Host github.com-key2
        HostName github.com
        User key2@example.com
        IdentityFile ~/.ssh/key2
```

`Host`에는 `github.com` 뒤에 구분할 수 있는 태그를 달았다. 이는 SSH 사용 시 계정 별로 Host를 다르게 함으로써 구분하여 사용할 수 있게 한다. 

다음으로 생성된 키는 `ssh agent`에 등록을 해줘야 사용이 가능하다.

```bash
ssh-add key1
ssh-add key2

# 등록된 키 목록 확인
ssh-add -l
# 등록된 키 전체 삭제
ssh-add -D
```

이제 `Host`에 설정한 값을 이용하여 GitHub 저장소를 clone 받아온다. 기본 경로가 `git@github.com:[user]/[repo]` 이지만, `git@github.com-key1:[user1]/[repo1]`, `git@github.com-key2:[user2]/[repo2]`와 같이 변경되어야 한다.

# 참고문헌
* [SSH를 통한 GitHub 연결](https://docs.github.com/ko/authentication/connecting-to-github-with-ssh)
* [SSH(Secure Shell)](https://www.ssh.com/academy/ssh)
* [Wikipedia Secure Shell](https://en.wikipedia.org/wiki/Secure_Shell)
* [GitHub 접속 용 SSH 키 만드는 방법](https://www.lainyzine.com/ko/article/creating-ssh-key-for-github/)
* [컴퓨터 한대로 github 여러 계정 사용하기](https://www.irgroup.org/posts/github-%EC%BB%B4%ED%93%A8%ED%84%B0-%ED%95%9C%EB%8C%80%EB%A1%9C-%EC%97%AC%EB%9F%AC-%EA%B3%84%EC%A0%95-%EC%82%AC%EC%9A%A9%ED%95%98%EA%B8%B0/)
* [[리눅스] ssh란?](https://velog.io/@hyeseong-dev/%EB%A6%AC%EB%88%85%EC%8A%A4-ssh%EB%9E%80)