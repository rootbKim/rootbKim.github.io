---
layout: note_page
title: Linux 명령어 모음
tags: [Linux]
category: "Dev"
---

리눅스에서 자주 사용되는 command를 알파벳 순으로 정리하였다.

## A

- alias : 명령어를 간소화하여 다른 이름으로 사용할 수 있도록 해주는 쉘 내부 명령어

```bash
# 현재 등록되어 있는 alias 목록
alias

# alias 등록
alias [alias_name]='[command_name]'
```

- apt-cache : 패키지 검색 도구

```bash
# 패키지 검색
apt-cache search [package_name]

# 패키지 정보 보기
apt-cache show [package_name]

# 패키지 의존성 확인
apt-cache depends [package_name]

# 패키지 역 의존성 확인
apt-cache rdepends [package_name]
```

- apt, apt-get : 데비안 계열의 리눅스에서 쓰이는 패키지 관리 명령어 도구

```bash
# 패키지 인덱스 업데이트(/etc/apt/source.list의 인덱스 정보 이용)
sudo apt-get update

# 설치된 패키지 업그레이드
sudo apt-get upgrade
sudo apt-get upgrade [package_name]

# 패키지 설치
sudo apt-get install [package_name]

# 패키지 재설치
sudo apt-get --reinstall install [package_name]

# 패키지 삭제
sudo apt-get remove [package_name]

# 패키지 완전 삭제(구성 파일 포함)
sudo apt-get purge [package_name]

# 패키지 소스코드 다운로드
sudo apt-get source [package_name]
```

## B

- bg : "background"를 나타내며, 작업을 백그라운드로 보내는 명령어, "foreground"에 해당하는 fg와 반대되는 명령어

## C

- cal : 캘린더를 띄우는 명령어

```bash
# 서버시간 기준의 연,월,일이 표시된 해당 월의 달력 표시
cal

# 해당 연도의 모든 월의 달력 표시
cal -y

# 특정 연 월 출력
cal 8 2021
```

- cat : 파일의 내용을 화면에 출력하거나 파일을 만듦

```bash
# 파일의 내용 출력
cat [fileName]
cat [fileName1] [fileName2] [fileName3] ...

# 파일의 내용 합치기(1,2,3번을 합쳐서 4번 파일을 만듦)
cat [fileName1] [fileName2] [fileName3] > [fileName4]

# 파일 내용 덧붙이기(2번의 끝에 1번을 덧붙임)
cat [fileName1] >> [fileName2]

# 새로운 파일을 만들기
cat > [new_file]
```

- cd : 해당 경로의 디렉토리로 이동함.

```bash
# path에 해당하는 위치로 이동
cd [path]

# 현재 디렉토리로 이동
cd .

# 한 단계 상위 디렉토리로 이동
cd ..

# 최상위 디렉토리로 이동
cd /
```

- chmod : 파일의 권한 변경. 숫자로 변경하는 방법만 간단하게 정리

```bash
# 모든 사용자의 모든 권한 제거
chmod 000 FILE

# 사용자(읽기+쓰기), 그룹(읽기+쓰기), 그 외 사용자(읽기)
chmod 664 FILE

# 사용자에게 모든 권한 추가
chmod 700 FILE

# 사용자(읽기+쓰기+실행), 그룹(읽기), 그 외 사용자(읽기)
chmod 744 FILE

# 사용자(읽기+쓰기+실행), 그룹(읽기+실행), 그 외 사용자(읽기+실행)
chmod 755 FILE

# 모든 사용자에 모든 권한 추가.
chmod 777 FILE
```

- chown : 파일이나 디렉터리의 소유자를 변경하는 명령어

```bash
# 파일이나 디렉토리의 소유자 변경
chown user /path/to/file_or_directory

# 파일이나 디렉토리의 소유자 및 그룹 변경
chown user:group /path/to/file_or_directory

# symbolic link의 소유자 변경
chown -h user /path/to/symboli_link

# symbolic link를 포함한 파일이나 디렉토리의 소유자와 그룹 변경
chown -hR user:group /path/to/file_or_dir
```

- cp : 파일을 복사하는 데 사용

```bash
# filename1을 복사하여 filename2를 만듦
cp [filename1] [filename2]

# filename을 복사하여 directory안에 filename을 만듦 (directory가 존재한다고 가정)
cp [filename] [directory]
```

## D

- date : 날짜 출력 명령어

```bash
# 현재 시간 출력
date

# 표준 시간 표시하기
date -u

# format 지정 출력(YYYY-MM-DD)
date "+%Y-%m-%d"

# format 지정 출력(YYYY-MM-DD 24시간 표시,분,초)
date "+%Y-%m-%d %H:%M:%S"

# format 지정 출력(12시간 표시,분,초 오전/오후 출력)
date "+%Y-%m-%d %I:%M:%S %p"

# unix time stamp
data +%s

# 시스템 시간 설정
sudo date  "+%Y-%m-%d %H:%M:%S" -s "20200412-13:24:50"
```

- df : 시스템 전체의 디스크 여유 공간 확인

```bash
# 디스크 공간 확인
df

# 모든 파일 시스템 출력
df -a

# total 추가
df --total

# 용량 크기로 출력
df -h
```

- dmesg : 시스템 부팅 메세지를 확인하는 명령어, 커널에서 출력되는 메세지를 일정 수준 기록하는 버퍼 역할을 수행, 커널 부팅 중에 에러가 났다면 어느 단계에서 에러가 났는지 범위를 좁히고 찾아내는데 사용

```bash
# 옵션 없이 커널의 로그 생성
sudo dmesg

# 모든 메세지 삭제 및 새로운 로그 생성
sudo dmesg -c

# grep으로 특정 로그 검색
sudo dmesg | grep [message name]

# level별 로그 검색
sudo dmesg --level [level name]

# facility별 로그 검색
sudo dmesg --f [facility name]

# 변경 사항 대기
sudo dmesg -w

# timestamp 추가
sudo dmesg -T

# 가독성 높이기
sudo dmesg -H

# color 추가
sudo dmesg -L

# THL 추가
sudo dmesg -THL
```

- dpkg : deb 패키지의 설치, 삭제, 정보 제공을 위해 사용하는 명령어

```bash
# 설치된 패키지 목록 확인
dpkg -l

# 해당 패키지로부터 설치된 파일 목록 확인
dpkg -L [package name]

# 해당 .deb 파일이 설치한 파일 목록 확인
dpkg -C [.deb name]

# 해당 패키지 정보 확인
dpkg -s [package name]

# 해당 .deb 파일의 정보 확인
dpkg -I [.deb name]

# 해당 파일명 또는 경로가 포함된 패키지 검색
dpkg -S [file name]

# 해당 패키지 삭제(설정파일은 남겨둠)
sudo dpkg -r [package name]

# 해당 패키지 삭제(설정파일 포함)
sudo dpkg -P [package name]

# .deb파일에 포함되어 있는 파일들을 지정된 디렉토리 초기화 후 압축해제
sudo dpkg -x [.deb name] [directory name]
```

- du : 디렉토리별 디스크 사용량 확인

```bash
# 모든 디렉토리 및 그 하위의 모든 디렉토리의 용량 확인
du

# 특정 디렉토리와 하위의 모든 디렉토리의 용량 확인
du [directory name]

# 선택한 디렉토리의 용량만 확인
du -s [directory name]

# 읽기 편한 단위로 용량 확인
du -h [directory name]

# 디렉토리의 바로 아래 디렉토리까지의 용량을 확인
du -sh [directory name]/*

# N 단계의 하위 디렉토리까지의 용량 확인
du -d N [directory name]

# 파일의 용량까지 확인
du -a [directory name]
```

## E

- echo : 문자열을 컴퓨터 터미널에 출력하는 명령어이다. 일반적으로 셸 스크립트와 배치 파일에서 화면이나 파일로 상황을 알리는 문자열을 출력할 때에 사용

```bash
# 문자열 출력
echo [string]

# 백슬래쉬 특수문자 인식
echo -e [string]

# 환경변수 출력
echo $[env var name]

# echo 결과 파일 저장
echo [string] >> [file name]
```

- exit : 현재 세션 종료

- export : 환경 변수 목록 확인, 환경 변수 값 설정

```bash
# 환경 변수 목록 확인
export

# 환경 변수 값 설정
export [env var name]=[value]
```

## F

- fg : "foreground"를 나타내며, 현재 백그라운드로 실행중인 명령어를 포그라운드작업으로 전환을 할때 사용하는 명령어

- find : 리눅스에서 파일 및 디렉토리를 검색할 때 사용하는 명령어

```bash
# 현재 디렉토리(.)의 파일 검색
find .

# 특정 디렉토리의 파일 검색
find . [directory name]

# 찾을 파일 이름 지정
find . -name "[file name]"

# 디렉토리만 검색
find . -type d

# 파일만 검색
find . -type f

# 파일 사이즈 지정
sudo find . -size [file size]
sudo find . -size +10M

# 빈 파일 검색
find . -empty

# 서브 디렉토리 검색 깊이 지정
find . -maxdepth

# 해당 파일보다 최근에 변경된 파일 검색
find . -newer [file name]

# 검색한 파일로 부가적인 작업 수행(검색된 파일이 {}위치에 입력되어 처리됨)
# 빈파일 정리
find . -empty -exec rm {} \;

# permission denied 없애기
find . 2>/dev/null
```

- free : 메모리 상태 확인

```bash
# 메모리 상태 확인
free

# KB 단위로 확인
free -k

# MB 단위로 확인
free -m

# GB 단위로 확인
free -g
```

## G

- grep : 입력으로 전달된 파일의 내용에서 특정 문자열을 찾고자할 때 사용하는 명령어

```bash
# 대소문자 구분 없이 검색
grep -i "[pattern]" [file name]

# 일치 횟수 표시
grep -c "[pattern]" [file name]

# 패턴과 일치하는 파일 표시
grep -l "[pattern]" *

# 전체 단어 단위로 검색
grep -w "[pattern]" [file name]

# 일치하는 패턴만 표시
grep -o "[pattern]" [file name]

# 줄 번호 표시
grep -n "[pattern]" [file name]

# 패턴과 일치하지 않은 행 표시
grep -v "[pattern]" [file name]

# 주어진 패턴으로 시작하는 행 표시
grep "^[pattern]" [file name]

# 주어진 패턴으로 끝나는 행 표시
grep "[pattern]$" [file name]
```

## H

- head : 텍스트로된 파일의 앞부분을 지정한 만큼 출력하는 명령어

```bash
# 앞에서부터 10행 출력
head [file name]

# 앞에서부터 N행 출력
head -n N [file name]

# 앞에서부터 N byte 출력
head -c N [file name]

# 출력된 내용 저장
head [file name 1] > [file name 2]

# 여러 개의 파일 출력
head [file name 1] [file name 2]

# 파일 제목 생략하고 출력
head -q [file name]
```

## I

- id : 현재 사용자의 실제 id와 유효 사용자 id, 그룹 id를 출력하며 내부 bash 변수인 $UID, $EUID, $GROUPS와 짝을 이룸.

- ifconfig : 네트워크 인터페이스를 설정하거나 확인하는 명령어. IP주소, 서브넷마스크, MAC주소, 네트워크 상태 등을 확인, 설정하는 명령어

```bash
sudo apt-get install net-tools

# ip 확인하기
ifconfig

# 특정 이더넷 이름에 ip 주소 설정
ifconfig [ethernet name] [ip address]

# 특정 이더넷 이름에 서브넷 마스크 설정
ifconfig [ethernet name] netmask [netmask address]

# 특정 이더넷 이름에 broadcast 설정
ifconfig [ehternet name] broadcast [broadcast address]
```

## J

- join : 두 파일을 의미있는 형태로 묶어 하나의 파일로 만드는 명령어로 공통으로 표시된 필드가 들어 있는 줄에 대응하여 합침.

```bash
# 두 파일의 공통된 내용을 보여줌
join [file name 1] [file name 2]

# 두 파일의 일치하지 않는 내용을 보여줌
join -v [file name 1] [file name 2]
```

## K

- kill : 프로세스에 특정한 signal을 보내는 명령어

```bash
# 프로세스 종료(pid 확인 : ps), 아래는 모두 같은 명령
kill [pid]
kill -15 [pid]
kill -TERM [pid]

# 시그널 종류 출력
kill -l
```

## L

- ll : ls -l 과 동일한 명령어

- ln : 링크파일을 만드는 명령어

```bash
# 하드 링크 파일 생성
ln [original file name] [hard link file name]

# 심볼링 링크 파일 생성
ln -s [original file name] [symbolic link file name]
```

- locale : 현재 설정된 locale(사용자 인터페이스에서 사용되는 언어, 지역 설정, 출력 형식 등을 정의하는 문자열)을 확인하는 명령어

```bash
# 현재 설정된 locale 확인
locale

# 가능한 locale 언어 목록 확인
locale -a

# locale 설정
export LANG=ko_KR.utf8
```

- ls : 현재 디렉토리 위치의 파일 목록을 조회

```bash
# 현재 디렉토리 위치의 파일 목록 조회
ls

# 숨겨진 파일 포함하여 조회
ls -a

# 퍼미션, 포함된 파일 수, 소유자, 그룹, 파일크기, 수정일자, 파일이름 출력
ls -l

# 파일 크기 순으로 정렬하여 출력
ls -S

# 알파벳의 역순으로 출력
ls -r

# 하위 디렉토리까지 출력
ls -R

# 사람이 보기 좋은 단위로 출력
ls -h

# 접근 시간 출력
ls -lu

# 변경 시간 출력
ls -lc
```

## M

- man : 각 종 명령어들의 자세한 사용법이나 매뉴얼을 볼 때 사용하는 명령어

```bash
# 명령어 메뉴얼 표시
man [command]
```

- mkdir : 새로운 디렉토리를 만드는 명령어

```bash
# directoryname의 새로운 디렉토리 생성
mkdir [directory name]

# 디렉토리를 만들 때, 하위 디렉토리까지 만들 때 사용
mkdir -p [directory name 1]/[directory name 2]

# 디렉토리 만들 때, 권한 지정
mkdir -m 700 [directory name]
```

- more : 리눅스에서 파일 내용을 확인하는 명령어들 중에 하나로, 파일을 읽어 화면에 화면 단위로 끊어서 출력하는 명령어

```bash
# 파일의 내용 확인
more [file name]

# n행씩 출력
more -n [file name]

# n행부터 출력
more +n [file name]

# 파일 출력을 more로 출력하기
ls -al | more
```

- mv : 파일을 이동시키는 명령어

```bash
# 파일 이름 변경(file name 1 --> file name 2)
mv [file name 1] [file name 2]

# 해당 파일을 특정 디렉토리로 이동
mv [file name] [directory name]

# 여러개의 파일을 특정 디렉토리로 이동
mv [file name 1] [file name 2] [directory name]

# 디렉토리 이름 변경(directory name 1 --> directory name 2)
mv [directory name 1] [directory name 2]
```

## N

- nslookup : name server 관련한 조회를 할 수 있는 명령어, 서버의 네트워크가 제대로 설정되었는지 확인하는 용도로 주로 사용

```bash
# IP 조회
nslookup google.com

# MX(Mail Record) 확인
nslookup -query=mx google.com

# CNAME 조회
nslookup -q=cname example.com

# TEXT 조회
nslookup -q=txt gmail.com

# NS 레코드로 DNS 목록 확인
nslookup -type=ns google.com

# Reverse DNS lookup
nslookup 209.132.183.181

# 특정 DNS 사용하여 조회
nslookup redhat.com 8.8.8.8
```

## P

- ps : 현재 실행 중인 프로세스 목록과 상태를 출력

```bash
# pid, cmd 등 기본적인 프로세스 목록 출력
ps

# 풀 포맷으로 출력
ps -f

# 긴 포맷으로 출력
ps -l

# 프로세스 번호가 1인 프로세스 출력
ps -p 1

# 계정이 apache인 프로세스 출력
ps -u apache

# 모든 프로세스 출력
ps -e
```

- pwd : 현재 작업 중인 디렉토리의 절대 경로를 출력하는 명령어

- ping : 네트워크 상태를 확인하려는 대상 컴퓨터를 향해 일정 크기의 패킷을 전송하는 명령어

```bash
# 네트워크 대역폭 혹은 속도에 따라 ping의 송신간격 결정
ping -A www.google.com

# 패킷의 갯수 걸정
ping -c 10 www.google.com

# 타임스탬프 출력
ping -D www.google.com

# 빠른 속도로 패킷 전송
ping -f www.google.com

# 패킷 전송 간격 조절
ping -i 0.2 www.google.com
```

## R

- rm : 파일 또는 디렉토리를 제거하는 명령어

```bash
# 특정 파일 제거
rm [file name]

# 모든 파일 제거
rm *

# 특정 디렉토리 제거
rm -r [directory name]

# 특정 디렉토리 강제 제거
rm -rf [directory name]

# 특정 디렉토리의 내용을 확인하며 제거
rm -ri [directory name]
```

- rmdir : 비어있는 디렉토리 삭제

```bash
# 특정 디렉토리 삭제
rmdir [directory name]

# 여러 개의 디렉토리 삭제
rmdir [directory name 1] [directory name 2]

# 상위 디렉토리 포함 사게
rmdir -p [directory name 1]/[directory name 2]
```

## S

- scp : ssh를 기반으로 한 파일 전송 프로토콜

```bash
# (복수) 파일을 원격지로 보낼 때
scp [filename1] [filename2] [remote_id]@[remote_ip]:[remote_locate]

# 여러 파일을 포함한 디렉토리를 원격지로 보낼 때
scp -r [directoryname] [remote_id]@[remote_ip]:[remote_locate]

# 파일을 원격지에서 로컬로 가져올 때
scp [remote_id]@[remote_ip]:[remote_locate] [local_locate]

# 복수의 파일을 원격지에서 로컬로 가져올 때
# 원격지 경로의 파일을 큰따옴표("")로 묶어주어야 함
scp [remote_id]@[remote_ip]:"[remote_locate1] [remote_locate2]" [local_locate]

# 여러 파일을 포함한 디렉토리를 원격지에서 로컬로 가져올 때
scp -r [remote_id]@[remote_ip]:[remote_locate] [local_locate]
```

- sh : 기본으로 지정된 쉘(shell)을 호출하는 명령어

- source : bash 명령어로, bash 쉘이 작동 중일 때만 동작함. filename 안의 환경설정 내용을 즉시 적용하기 위해 사용됨

```bash
source [filename]
```

- ssh : Secure Shell의 약자로 네트워크 상의 다른 컴퓨터에 로그인하거나 원격 시스템에서 명령을 실행하고 다른 시스템으로 파일을 복사할 수 있도록 해주는 응용 프로그램 또는 프로토콜

```bash
sudo apt-get install ssh

# 특정 사용자의 ip 주소에 접속
ssh [user name]@[ip address]

# 특정 포트에 접속
ssh -p [port num] [user name]@[ip address]
```

- sudo : root 권한을 이용하여 명령어를 실행할 때 사용

## T

- tail : 파일의 마지막 부분을 출력하는 프로그램

```bash
# 뒤에서부터 10행 출력
tail [file name]

# 뒤에서부터 N행 출력
tail -n N [file name]

# 뒤에서부터 N byte 출력
tail -c N [file name]

# 출력된 내용 저장
tail [file name 1] > [file name 2]

# 여러 개의 파일 출력
tail [file name 1] [file name 2]

# 파일 제목 생략하고 출력
tail -q [file name]
```

- tar : tar 혹은 tar.gz로 압축을 하거나 압축을 해제하는 명령어

```bash
# tar로 압축하기
tar -cvf [tar name].tar [directory name]

# tar 압축 해제
tar -xvf [tar name].tar

# tar.gz로 압축하기
tar -zcvf [tar name].tar.gz [directory name]

# tar.gz 압축 해제
tar -zxvf [tar name].tar
```

- top : CPU의 사용률을 확인하는 명령어

- touch : 0바이트 파일을 생성함

```bash
# filename의 0바이트 파일을 생성
touch [filename]
```

- tree : 디렉토리의 트리 구조를 볼 수 있음

```bash
sudo apt-get install tree
```

## U

- uname : 시스템의 정보(커널명, 호스트명, 커널 릴리즈, 커널 버전, 머신, 프로세스 하드웨어 플랫폼, OS)를 확인하는 명령어

```bash
# 시스템의 모든 정보 출력
uname -a

# 커널명확인
uname -s

# 사용중인 네트워크 호스트 이름 확인
uname -n

# 커널 릴리즈 확인(운영체제 배포 버전)
uname -r

# 커널 버전 확인
uname -v

# 시스템 하드웨어 타입 정보 확인
uname -m

# 프로세서 정보 확인
uname -p

# 시스템의 하드웨어 플랫폼 정보 확인
uname -i

# 운영체제 정보 확인
uname -o
```

- uptime : 시스템이 실행되고 난 후 부터 지금까지의 시간과 시스템에 로그인 된 사용자 수 그리고 시스템 부하율을 표시하는 명령어

```bash
# 현재시간, 시스템 실행시간, 로그인된 사용자 수, 부하율을 표시
uptime

# 현재까지 운영된 시간 출력
uptime -p

# 부팅이 시작된 시간 출력
uptime -s
```

## V

- vi : vi 편집기 사용

```bash
# 새로운 문서 편집
vi [file name]
```

## W

- w : 서버에 접속한 사용자의 접속정보 및 작업 정보 내용을 확인하는 명령어

```bash
# 헤더 정보를 뺀 간략한 정보 출력
w -h

# 로그인 시간, JCPU, PCPU를 제외한 간략한 정보 출력
w -s

# FROM 필드의 값 제외한 간략한 정보 출력
w -f

# FROM 필드에 호스트 이름 대신 IP 주소 출력
w -i
```

- wget : Web get의 약어로 웹 상의 파일을 다운로드할 때 사용하는 명령어

- whereis : 명령어의 실행 파일 절대 경로와 소스코드, 설정 파일 및 매뉴얼 페이지를 찾아 출력하는 명령어

```bash
# 특정 명령어의 경로, 소스코드, 설정 파일 및 메뉴얼 페이지 출력
whereis [command]
```

- which : 명령어의 위치를 찾아주는 명령어

```bash
# 특정 명령어의 경로 출력
which [command]
```

- whoami : 현재 로그인한 사용자 ID를 출력하는 명령어, id명령어에 -un 옵션을 준 것과 같은 효과

- whois : 도메인 이름의 소유자가 누구인지 알려주는 명령어

```bash
# 도메인 이름을 이용한 소유자 확인
whois www.google.com

# IP를 이용한 소유자 확인
whois [IP address]
```
