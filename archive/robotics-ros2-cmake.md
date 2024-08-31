---
layout: archive
title: ROS2 CMakeLists.txt
tags: [ROS2]
category: "Robotics"
---

ROS2에서 패키지를 생성하고 빌드할 때, 사용되는 CMake 빌드 설정 파일인 CMakeLists.txt에 대해서 알아보고, CMakeLists.txt에서 기본적으로 사용되는 메소드들에 대해서 간단하게 알아본다.

# 1. CMakeLists.txt란?

`CMakeLists.txt`는 ROS 패키지의 C++ 프로젝트들을 빌드할 때 사용되는 CMake 빌드 설정 파일이다. 해당 설정 파일에 빌드에 필요한 내용을 기록하고, 빌드 시 해당 내용을 기반으로 빌드를 수행하게 된다. `CMake`에 대한 내용은 다른 페이지에서 다루기로 하며, 여기서는 ROS에서 사용되는 기본적인 메소드들에 대해서 정리하고자 한다. 또한 해당 페이지는 ROS 1.0의 `CMakeLists.txt`와 비교하며, `ROS 2.0`의 `CMakeLists.txt`의 특징과 세부사항에대해 기술하였다.

# 2. CMake 버전

패키지를 빌드하기 위해 최소로 필요한 `CMake` 버전을 입력한다. `ROS 1.0`의 `catkin`에서는 최소 `2.8.3` 이상을 필요로 하며, `ROS 2.0`에서는 그보다 더 높은 버전인 `3.5.0` 버전을 사용한다.

```cmake
cmake_minimum_required(VERSION 3.5.0)
```

# 3. Package Name

패키지 명을 `project` 함수를 통해 전달한다. 현재 패키지의 폴더 명과 동일해야 한다.

```cmake
project([PACKAGE_NAME])
```

# 4. Set C++ version

해당 명령은 `ROS 1.0`보다는 `ROS 2.0` 버전에서 고려되는 점으로, `ROS 2.0`은 최소 `C++ 14 standard` 를 기반으로 하기 때문에, 컴파일러가 `C++ 14` 또는 그 이상의 버전으로 컴파일하도록 설정해주어야 한다.
다음은 `C++ 14 standard`로 컴파일되도록 설정하는 부분이다.

```cmake
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
```

# 5. 빌드에 필요한 CMake 패키지 찾기

`find_package()` 함수를 이용하여 패키지 빌드를 위해 필요한 다른 의존 패키지들을 입력한다.

`ROS 2.0`의 경우 `ament_cmake` 패키지를 통해 빌드되므로, `ament_cmake` 패키지를 추가한다.

```cmake
find_package(ament_cmake REQUIRED)
```

또한, `ROS 2.0`의 경우, `ROS 1.0`과 다르게 `COMPONENTS` 인자를 사용하지 않고, 각각의 패키지들을 다음 예시처럼 `find_package()` 함수로 명시해주어야 한다.

```cmake
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
...
```

그 이외의 추가적으로 필요한 다른 의존성 패키지들을 추가하면 된다.

# 6. 메세지, 서비스, 액션 타겟

`ROS 2.0`에서는 메세지, 서비스, 액션 타겟을 생성하기 위해서 `rosidl_generate_interfaces()` 함수를 이용한다. 아래는 `set()`함수를 이용해서 `msg_files`, `srv_files`, `action_files`를 생성하여 추가하고, 다른 필요한 `DEPENDENCIES`를 추가한 예제이다.

```cmake
# msg_files
set(msg_files
  "msg/[MessageName1].msg"
  "msg/[MessageName2].msg"
  "msg/[MessageName3].msg"
  ...
)
# srv_files
set(srv_files
  "srv/[ServiceName1].srv"
  "srv/[ServiceName2].srv"
  "srv/[ServiceName2].srv"
  ...
)
# action_files
set(action_files
  "action/[ActionName1].action"
  "action/[ActionName2].action"
  "action/[ActionName3].action"
  ...
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs ...
)
```

또한, `find_package()`함수를 이용하여 `rosidl_default_generators`를 추가해야 한다.

```cmake
find_package(rosidl_default_generators REQUIRED)
```

# 7. 빌드 타겟 정의

빌드 타겟은 라이브러리와 실행 파일을 빌드 타겟으로 정의할 수 있다. 라이브러리는 `add_library()`함수로 정의하며, 실행 파일은 `add_executable()`함수르 정의한다.

## 7.1 실행 파일 추가

빌드할 대상인 실행 파일을 추가하기 위해 `add_executable()` 함수를 사용한다.

```cmake
add_executable(myProgram src/main.cpp src/위 예제는 3 개의 소스 파일(src/main.cpp, src/some_file.cpp, src/another_file.cpp)을 빌드하여 `myProgram이라는 이름의 실행 파일을 만들어낸다.
```

위 예제는 3 개의 소스 파일(src/main.cpp, src/some_file.cpp, src/another_file.cpp)을 빌드하여 myProgram이라는 이름의 실행 파일을 만들어낸다.

## 7.2 라이브러리 타겟 추가

`add_library()` 함수를 이용하여 빌드할 라이브러리를 기술한다.

```cmake
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
```

## 7.3 헤더파일 추가

아래의 명령으로 빌드에 필요한 헤더파일들을 추가할 수 있으며, 그 이외에도 앞서 `find_package()` 호출 시 생성된 `*_INCLUDE_DIRS` 환경 변수를 통해 추가적인 헤더파일들을 추가할 수 있다.

```cmake
target_include_directories(my_target
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
    ${[PACKAGE_NAME]_INCLUDE_DIRS}
    ...
```

## 7.4 DEPENDENCIES 추가

의존성 추가 방법에는 `ament_target_dependencies()` 함수를 이용하는 방법과 `target_link_libraries()` 함수를 이용하는 방법이 있다. 의존성 추가를 위해, 앞서 `find_package()`를 통해 의존성을 추가할 패키지를 추가해야 한다.

우선, `ament` 매크로인 `ament_target_dependencies()`는 의존성이 필요한 헤더, 라이브러리 및 그와 관련된 `dependencies`들을 가져온다. 다음은 `my_target`이라는 `link` 이름에 `Eigen3`의 의존성을 추가하는 방법이다.

```cmake
find_package(Eigen3 REQUIRED)
ament_target_dependencies(my_target Eigen3)
```

다음으로 `target_link_libraries()`는 라이브러리의 `namespace`를 명명한다. 다음은 `my_target`이라는 `link` 이름에 `Eigen3`의 의존성을 추가하는 방법이다.

```cmake
find_package(Eigen3 REQUIRED)
target_link_libraries(my_target Eigen3::Eigen)
```

`ament_target_dependencies()`는 모든 `dependency`의 순서가 올바르게 지정되지만, `target_link_libraries()`는 `dependency`의 순서가 올바르지 않을 수도 있다.

그리고, `ament package`가 아닌 경우에는 `target_link_libraries()`를 이용해 라이브러리를 추가해야 한다. 물론, `ament package`가 있는 경우에도, `ROS 1.0`에서처럼 다음의 방법으로 추가할 수 있다.

```cmake
target_link_libraries(target ${Boost_LIBRARIES})
```

# 8. install

다음 예시는 `workspace` 내의 `install` 폴더에 설치할 디렉토리를 설정한다. `DIRECTORY` 인자 뒤에 설치할 디렉토리들을 선정하고, `DESTINATION` 인자 뒤에는 설치 경로, 즉 공유할 경로를 설정하면 된다.

```cmake
install(
  DIRECTORY include/
  DESTINATION include
)
```

다음 예시는 `install` 함수를 이용하여 라이브러리를 설치하는 명령이다. `ARCHIVE` 및 `LIBRARY` 파일은 `lib` 폴더에 설치되고, `RUNTIME binary` 파일은 `bin` 폴더에 설치되며, 설치된 헤더의 경로가 포함된다.

```cmake
install(
  TARGETS my_library
  EXPORT export_my_library
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
```

# 참고문헌

- [ROS 2.0 foxy Ament CMake Documentation](https://docs.ros.org/en/foxy/Guides/Ament-CMake-Documentation.html)
- [CMake Main page](https://cmake.org/)
- [Migration Guide from ROS 1.0 to ROS 2.0](https://docs.ros.org/en/foxy/Contributing/Migration-Guide.html)
