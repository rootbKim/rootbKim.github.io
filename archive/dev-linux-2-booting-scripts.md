---
layout: note_page
title: systemd와 우분투 서버 부팅 스크립트
tags: [Linux, Systemd]
category: "Dev"
---

우분투 리눅스에서 systemd를 이용한 부팅 스크립트를 작성하고, 사용하는 방법에 대해 다룬다.

# systemd

systemd(system daemon)은 Unix 시스템이 부팅후에 가장 먼저 생성된 후에 다른 프로세스를 실행하는 init 역할을 대체하는 데몬으로, Red Hat 에서 주도적으로 개발을 시작했고 지금은 RHEL/CentOS 와 Ubuntu 나 Arch 등 대부분의 리눅스 시스템에 공식적으로 채택됐다.

systemd(system daemon)는 1번 프로세스 ID를 갖고 있으며 부팅부터 서비스관리 로그관리 등을 담당하며, 부팅시에 병렬로 실행되어서 부팅속도가 빠르다. 오래전에 사용하던 init방식은 부팅 과정에서 단계적으로 run-level을 올려가며 해당 run-level에 포함된 스크립트들을 순차적으로 실행시키기 때문에 설정이 복잡하고, 속도도 느렸다.

반면 systemd는 다음과 같은 장점을 갖는다.

1. 적극적인 병렬처리 기능
2. 소켓과 D-Bus 활성화
3. on-demand 데몬 실행
4. 프로세스 트래킹
5. 파일시스템 마운트 및 마운트 관리
6. 의존성 기반의 서비스 컨트롤 로직
7. SysV init 스크립트에 대한 호환성
8. 로깅 데몬 제공
9. 기본적인 시스템 설정 컨트롤 유틸리티 제공
10. 다양한 네트워크 관련 데몬

# 스크립트 작성

부팅 시 실행할 스크립트를 작성한다. 스크립트는 `.sh`를 확장자로 가진다. 예를 들어 부팅 시 `ros_ws`에 있는 `boot_ros` 패키지의 `boot_ros.launch.py`라는 런치파일을 실행하는 `boot_ros.sh`스크립트를 홈 폴더 위치에 작성하였다.

```bash
#!/bin/bash

source /opt/ros/foxy/setup.bash
source /home/rootb/ros_ws/install/setup.bash

sleep 3
ros2 launch boot_ros boot_ros.launch.py
```

다음으로 작성한 스크립트에 대한 실행 권한을 부여해준다.

```bash
sudo chmod 755 /home/rootb/boot_ros.sh
```

# 서비스 작성

`systemd`에 위에서 작성한 스크립트를 등록해야한다. 이를 위해 `/etc/systemd/user` 위치에 `.service`를 확장자로 갖는 서비스 파일을 생성한다. `/etc/systemd/user` 위치에 저장하여 `user` 권한으로 서비스를 실행할 수 있게 하였다.

> `user` 권한 으로 실행하지 않으면 ROS 관련 패키지 실행 시 `rcl_logger`와 관련된 에러가 발생

다음은 위에서 작성한 `boot_ros.sh` 스크립트를 등록한 `boot_ros.service`이다.

```bash
# /etc/systemd/user/boot_ros.service
[Unit]
Description=ROS Boot
DefaultDependencies=no
Before=shutdown.target

[Service]
Type=oneshot
ExecStart=/home/rootb/boot_ros.sh
TimeoutStartSec = 0

[Install]
WantedBy=default.target
```

> Unit 파일의 세부 작성 방법은 다음 [링크](https://www.digitalocean.com/community/tutorials/understanding-systemd-units-and-unit-files)를 참조한다.

# 서비스 등록

`systemd`에 작성한 service 파일을 등록한다. 이때 `--user` 태그를 걸어야 `/etc/systemd/user`에 만든 service에 접근할 수 있다.

```bash
systemctl --user daemon-reload
systemctl --user enable boot_ros.service
systemctl --user start boot_ros.service
```

실행 중인 service를 확인하기 위해 다음의 명령을 사용할 수 있다.

```bash
# 실행 중인 service 목록 확인
systemctl --user list-units --type=service

# service 상태 확인
systemctl --user status boot_ros.service
```

# 서비스 업데이트

만약 코드를 수정하고 빌드를 했다면, 재부팅을 해서 적용을 할 수도 있지만, 명령을 통해 service를 재시작할 수 있다.

```bash
systemctl --user restart boot_ros.service
```

또한 다음의 명령어로 service를 중지, 시작시킬 수 있다.

```bash
systemctl --user stop boot_ros.service
systemctl --user start boot_ros.service
```

# 참고문헌

- [[Linux] 부팅시 스크립트 자동 실행시키기(rc.local)](https://joonyon.tistory.com/entry/Linux-%EB%B6%80%ED%8C%85%EC%8B%9C-%EC%8A%A4%ED%81%AC%EB%A6%BD%ED%8A%B8-%EC%9E%90%EB%8F%99-%EC%8B%A4%ED%96%89%EC%8B%9C%ED%82%A4%EA%B8%B0rclocal)
- [리눅스에서 부팅 시 원하는 ROS 프로그램을 자동으로 실행하는 방법](https://injae-kim.github.io/robot_operating_system/2020/04/25/ros-automatically-execute-program-on-booting.html)
- [systemd에 유저 권한으로 서비스 추가하기](https://blog.majecty.com/wikis/2019-01-13-a-systemd-user.html)
- [systemd: Enabled but inactive??](https://www.nemonein.xyz/2020/09/4245/)
- [service in systemd user mode inactive dead](https://unix.stackexchange.com/questions/698463/service-in-systemd-user-mode-inactive-dead)
- [2. Systemd 기본 개념잡기 (1/2)](https://www.kernelpanic.kr/17)
- [Understanding Systemd Units and Unit Files](https://www.digitalocean.com/community/tutorials/understanding-systemd-units-and-unit-files)
