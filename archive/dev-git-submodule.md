---
layout: archive
title: Git Submodule
tags: [Git]
category: "Dev"
---

프로젝트를 수행하다 보면 다른 프로젝트를 함께 사용해야 하는 경우가 종종 있다. 함께 사용할 다른 프로젝트는 외부에서 개발한 라이브러리라던가 내부 여러 프로젝트에서 공통으로 사용할 라이브러리일 수 있다. 이런 상황에서 자주 생기는 이슈는 두 프로젝트를 서로 별개로 다루면서도 그 중 하나를 다른 하나 안에서 사용하는 경우에 서브모듈을 사용한다.

# 1. 서브모듈 사용하기

## 1.1 서브모듈 추가하기

작업할 Git 저장소에 미리 준비된 리모트 Git 저장소를 `git submodule add <Submodeul GIT Remote URL>` 명령어를 이용하여 서브모듈로 추가할 수 있다.

다음은 DbConnector 이라는 서브모듈을 현재 워크스페이스에 서브모듈로 추가한 것이다.

```bash
git submodule add https://github.com/chaconinc/DbConnector
git submodule add -b <Branch Name> https://github.com/chaconinc/DbConnector # 특정 브랜치 이름 지정
```

기본적으로 서브모듈은 프로젝트 저장소의 이름으로 디렉토리를 만든다. 예제에서는 “DbConnector” 라는 이름으로 만든다. 명령의 마지막에 원하는 이름을 넣어 다른 디렉토리 이름으로 서브모듈을 추가할 수도 있다.

서브모듈을 추가하면 `.gitmodules` 파일이 만들어졌다. 이 파일은 서브디렉토리와 하위 프로젝트 URL의 매핑 정보를 담은 설정파일이다.

```bash
[submodule "DbConnector"]
    path = DbConnector
    url = https://github.com/chaconinc/DbConnector
```

서브모듈 개수만큼 이 항목이 생긴다. 이 파일도 .gitignore 파일처럼 버전을 관리한다. 다른 파일처럼 Push 하고 Pull 한다. 이 프로젝트를 Clone 하는 사람은 .gitmodules 파일을 보고 어떤 서브모듈 프로젝트가 있는지 알 수 있다.

> 서브모듈로 추가된 저장소는 해당 디렉토리 아래의 파일 수정사항을 직접 추적하지 않는다. 대신 서브모듈 디렉토리를 통째로 특별한 커밋으로 취급하며, 서브모듈 내의 내용이 수정되면, 커밋 ID로 변경 사항을 관리한다.

## 1.2 서브모듈 포함한 프로젝트 Clone

서브모듈을 포함하고 있는 프로젝트를 Clone 하면 기본적으로 서브모듈은 빈 디렉토리로 Clone 된다.

먼저 `git submodule init` 명령으로 서브모듈 정보를 기반으로 로컬 환경설정 파일을 준비한다.

```bash
git submodule init
```

이후 `git submodule update` 명령으로 서브모듈의 리모트 저장소에서 데이터를 가져오고 서브모듈을 포함한 프로젝트의 현재 스냅샷에서 Checkout 해야 할 커밋 정보를 가져와서 서브모듈 프로젝트에 대한 Checkout을 한다.

```bash
git submodule update
```

위 과정을 한 번에 실행하려면 메인 프로젝트를 Clone 할 때 `git clone` 명령 뒤에 `--recurse-submodules` 옵션을 붙이면 서브모듈을 자동으로 초기화하고 업데이트한다.

# 2. 서브모듈 포함한 프로젝트 작업

## 2.1 서브모듈 업데이트하기

서브모듈 프로젝트를 최신으로 업데이트하려면 서브모듈 디렉토리에서 `git fetch` 명령을 실행하고 `git merge` 명령으로 Upstream 브랜치를 Merge한다. 여기서 커밋하면 서브모듈은 업데이트된 내용으로 메인 프로젝트에 적용된다. 다른 사람들이 업데이트하면 적용된다. 또한 `git submodule update --remote` 명령을 실행하면 Git이 알아서 서브모듈 프로젝트를 Fetch 하고 업데이트한다. 이 명령은 기본적으로 서브모듈 저장소의 `master` 브랜치를 Checkout 하고 업데이트를 수행한다.

```bash
git submodule update --remote # 모든 서브모듈 업데이트
git submodule update --remote DbConnector
```

만약 `master` 브랜치가 아닌 특정 브랜치를 업데이트하고 싶다면 `.gitmodules` 파일에서 설정을 바꾸면 된다. 다른 [gitmodules](https://git-scm.com/docs/gitmodules)은 여기서 확인한다.

```bash
[submodule "DbConnector"]
    path = DbConnector
    url = https://github.com/chaconinc/DbConnector
    branch = main # 해당 내용을 추가하고, 원하는 브랜치 입력
```

## 2.2 서브모듈 관리하기

서브모듈 안에서 수정사항을 추적하려면 다른 작업이 좀 더 필요하다. 서브모듈이 브랜치를 추적하게 하려면 할 일이 두 가지다. 우선 각 서브모듈 디렉토리로 가서 추적할 브랜치를 Checkout 하고 일을 시작해야 한다. 이후 서브모듈을 수정한 다음에 `git submodule update --remote` 명령을 실행해 Upstream 에서 새로운 커밋을 가져온다. 이 커밋을 Merge 하거나 Rebase 하는 것은 선택할 수 있다.

먼저 서브모듈 디렉토리로 가서 브랜치를 Checkout 하자.

```bash
git checkout stable
```

여기서 “Merge” 를 해보자. update 명령을 쓸 때 `--merge` 옵션을 추가하면 Merge 하도록 지정할 수 있다. 아래 결과에서 서버로부터 서브모듈의 변경 사항을 가져와서 Merge 하는 과정을 볼 수 있다.

```bash
git submodule update --remote --merge
```

DbConnector 디렉토리로 들어가면 새로 수정한 내용이 로컬 브랜치 stable 에 이미 Merge 된 것을 확인할 수 있다. 이제 다른 사람이 DbConnector 라이브러리를 수정해서 Upstream 저장소에 Push 한 상태에서 로컬의 DbConnector 라이브러리를 수정한 상태에서 update를 다음과 같이 할 수 있다.

```bash
git submodule update --remote --rebase
```

`--rebase` 옵션이나 `--merge` 옵션을 지정하지 않으면 Git은 로컬 변경사항을 무시하고 서버로부터 받은 해당 서브모듈의 버전으로 Reset을 하고 Detached HEAD 상태로 만든다. 이 경우 Reset이 된 서브모듈 디렉토리로 가서 작업하던 브랜치를 Checkout 하고 직접 `origin/stable` (아니면 원하는 어떠한 리모트 브랜치든)을 Merge 하거나 Rebase 하면 된다.

```bash
git submodule update --remote
```

만약 업데이트 명령을 실행했을 때 Upstream 저장소의 변경 사항과 충돌이 나면 알려주므로, 직접 서브모듈에 가서 충돌을 해결해주면 된다.

## 2.3 서브모듈 수정 사항 공유하기

서브모듈의 변경사항을 Push 하지 않은 채로 메인 프로젝트에서 커밋을 Push 하면 안 된다. 변경 사항을 Checkout 한 다른 사람은 서브모듈이 의존하는 코드를 어디서도 가져올 수 없는 상황이 돼 곤란해진다. 서브모듈의 변경사항은 우리의 로컬에만 있다.

이런 불상사가 발생하지 않도록 하려면 메인 프로젝트를 Push 하기 전에 서브모듈을 모두 Push 했는지 검사하도록 Git에게 물어보면 된다. `git push` 명령에 `--recurse-submodules` 옵션을 주고 이 옵션의 값으로 `check` 나 `on-demand` 를 설정한다. `check` 는 간단히 서브모듈의 로컬 커밋이 Push 되지 않은 상태라면 현재의 Push 명령도 실패하도록 하는 옵션이다.

```bash
git push --recurse-submodules=check
```

위 명령어로 서브모듈이 Push 되지 않은 상황을 해결하기 위한 가장 단순한 방법은 각 서브모듈 디렉토리로 가서 직접 일일이 Push를 해서 외부로 공유하고 나서 메인 프로젝트를 Push 하는 것이다.

`on-demand` 옵션은 Git이 Push를 대신 시도한다. Git이 메인 프로젝트를 Push 하기 전에 DbConnector 모듈로 들어가서 Push를 한다. 모종의 이유로 서브모듈 Push에 실패한다면 메인 프로젝트의 Push 또한 실패하게 된다.

```bash
git push --recurse-submodules=on-demand
```

# 참고문헌

- [Pro Git-서브모듈](https://git-scm.com/book/ko/v2/Git-%EB%8F%84%EA%B5%AC-%EC%84%9C%EB%B8%8C%EB%AA%A8%EB%93%88)
