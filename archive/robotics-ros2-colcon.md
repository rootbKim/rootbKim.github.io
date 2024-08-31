---
layout: archive
title: ROS2 COLCON
tags: [ROS2]
category: "Robotics"
---

ROS2에서 사용되는 빌드 툴인 colcon에 대해서 간단하게 그 개요에 대해서 정리하고, 자주 사용하는 colcon CLI를 정리한다.

# 1. Colcon 이란?

`colcon`은 command line tool로, 빌드, 테스트, 다수의 소프트웨어 패키지를 사용하는데 workflow를 개선하는데 사용된다. `colcon`은 이런 과정을 자동화하고, 명령을 처리하고, 패키지를 사용하도록 환경을 설정한다.

ROS 2.0에서는 `ament_cmake`를 이용하여 패키지들을 빌드하는데, `ament_cmake`의 사용의 불편함으로 인해, `catkin`이나 `ament` 등의 빌드 방식을 범용적으로 빌드할 수 있는 툴로써 `colcon`을 이용하여 패키지를 빌드한다.

ROS의 workspace는 `src` 폴더 내부에 ROS 패키지들이 들어가게 된다. 이 패키지들을 빌드하기 위하여 workspace 위치(`src`의 상위 폴더)에서 `colcon build`를 하게 되면, `build`, `install`, `log` 폴더들이 생성된다.

- `build` : CMake와 같이 빌드에 필요한 intermediate 파일들이 저장되는 공간으로, 빌드되는 패키지들의 폴더들이 생성됨.
- `install` : 각 패키지들이 설치되는 폴더로, 각 패키지별로 폴더가 생성됨. `catkin`으로 빌드하면 `devel` 폴더가 생성되었으나 `colcon`으로 빌드하게 되면 `install` 폴더가 생성됨.
- `log` : `colcon` 호출에 대한 로그 정보가 기록됨.

빌드 이후에 `install` 폴더에 설치된 패키지들을 ROS 패키지와 함께 사용하기 위해서는 source 명령을 통해 `install` 폴더 내부에 생성된 `setup.bash` 의 환경 설정 내용을 가져와야 한다.

```bash
source ~/[workspace_name]/install/setup.bash
```

# 2. ROS 프로젝트를 진행하며 사용되는 colcon CLI

다음은 `ROS` 프로젝트를 진행하면서 주로 사용되는 build 관련된 `command line arguments`를 정리하였다.

- `--symlink-install`
  빌드하는 파일들을 복사하지 않고 링크로 연결하여 수정이 용이하도록 하는 명령으로, 해당 명령으로 빌드된 python 파일의 코드는 첫 빌드 이후에 내용이 수정되어도 빌드를 다시 하지 않고도 사용할 수 있다.

- `--packages-select [package_name [package_name ...]]`
  `package_name` 패키지만 빌드한다.

- `--packages-up-to [package_name [package_name ...]]`
  패키지 내부에 dependency에 의해 먼저 빌드되어야 하는 패키지가 있을 때 사용되며, `package_name` 패키지 내부에 걸려있는 dependency 패키지들과 `package_name` 패키지를 빌드한다.

- `--cmake-args [*[* ...]]`
  CMake 과정에서 관련 arguments를 입력할 수 있다.

# 참고문헌

- [Colcon 관련 ROS 2.0 foxy 설치 페이지](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html)
- [Build tools 에 대한 비교 및 정리 문서](https://design.ros2.org/articles/build_tool.html)
- [Colcon github pages](https://github.com/colcon)
- [Colcon gitbook pages](https://colcon.readthedocs.io/en/released/#)
