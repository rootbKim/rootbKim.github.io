---
layout: post
title: ROS2 Costmap2D Layers
feature-img: "assets/img/posts/ros.png"
thumbnail: "assets/img/posts/ros.png"
tags: [ROS2, Nav2]
---

ROS2에서 사용되는 Nav2의 Costmap2D Plugin에 대해 설명하고, Custom Costmap2D Layer를 만드는 방법에 대해서 정리한다.

## 1. Costmap2D Plugin 만들기

Costmap2D Plugin의 예제로 [nav2_gradient_costmap_plugin](https://github.com/ros-planning/navigation2_tutorials/tree/master/nav2_gradient_costmap_plugin)을 참조한다.

## 1.1 Costmap2D Layer 패키지 구조

기본적으로 costmap plugin은 다음과 같은 구조를 갖는다. 이는 plugin 형태로 만들고 내보내기 위해 필요한 것들로 구성되어있는데, 각각에 대한 자세한 내용은 뒤에서 다룬다.

```bash
nav2_gradient_costmap_plugin
├── CMakeLists.txt
├── gradient_layer.xml
├── include
│   └── nav2_gradient_costmap_plugin
│       └── gradient_layer.hpp
├── package.xml
└── src
    └── gradient_layer.cpp
```

## 1.2 Costmap2D 코드 구현

플러그인 클래스는 `nav2_costmap_2d::Layer` 클래스를 상속받는다.

```cpp
namespace nav2_gradient_costmap_plugin
{
class GradientLayer : public nav2_costmap_2d::Layer
{
  // GradientLayer 클래스 정의
};
}
```

`nav2_costmap_2d::Layer` 클래스는 가상 함수를 갖기 때문에, 해당 가상 함수들을 모두 정의하고 구현해야 한다. 이 가상 함수들은 `LayeredCostmap`에서 실행된다.

가상 함수의 목록은 다음과 같다.

<img src="/assets/img/posts/230107_costmap_functions.png">

### 1.2.1 onInitailize()

`onInitalize()`는 해당 플러그인이 초기화 되고 호출이 되며, 이 함수 내에서 필요한 초기화들이 이루어진다. 해당 플러그인에서 사용되는 ROS Topic, Service나 Parameter를 정의하고 초기화한다.

```cpp
  /** @brief This is called at the end of initialize().  Override to
   * implement subclass-specific initialization.
   *
   * tf_, name_, and layered_costmap_ will all be set already when this is called.
   */
  virtual void onInitialize() {}
```

`need_recalculation_` 은 일반적으로 재계산이 필요한지 여부를 알려주는 플래그로 사용된다.

```cpp
  need_recalculation_ = false;
```

### 1.2.2 updateBounds()

`updateBounds()`는 Costmap에서 cost를 부여할 영역을 주기적으로 업데이트한다. 전체 영역을 업데이트하면 계산량이 많아지기 때문에 적절한 영역을 설정해주어야 한다. 입력 인자로 로봇의 위치(x, y, yaw)와 영역의 최소 x,y 좌표와 영역의 최대 x,y 좌표를 받는다.

업데이트는 보통 `need_recalculation_`이 `true`인 경우 수행하고, 그렇지 않은 경우에는 기존에 저장된 값을 그대로 사용한다.

```cpp
  /**
   * @brief This is called by the LayeredCostmap to poll this plugin as to how
   *        much of the costmap it needs to update. Each layer can increase
   *        the size of this bounds.
   *
   * For more details, see "Layered Costmaps for Context-Sensitive Navigation",
   * by Lu et. Al, IROS 2014.
   */
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) = 0;
```

### 1.2.3 updateCosts()

`updateBounds()`로 업데이트된 영역 내의 cost를 주기적으로 계산하고, 계산된 cost를 costmap에 반영하는 함수이다. 입력 인자로 영역의 경계 정보와 `master_grid`라는 costmap 결과를 반영할 참조자 변수가 있다. `matser_grid`는 다음 업데이트 방법 4 가지(`updateWithAddition()`,`updateWithMax()`, `updateWithOverwrite()`, `updateWithTrueOverwrite()`) 중 하나의 옵션으로 영역 범위 내에 cost를 계산하여 업데이트 된다.

> `master_array` 포인터가 `master_grid`의 결과와 연결되어 있어, `master_array`를 사용하면 이전에 계산된 다른 layer와 병합하지 않고 독립적으로 계산하게 된다. 이후에 `CostmapLayer`에서 `updateWithTrueOverwrite()`를 호출하여 다른 layer와 병합하는 방식이 된다.
>
> ```cpp
> unsigned char * master_array = master_grid.getCharMap();
> ```

> `master_array`를 이용하지 않고, 이전 layer에서 작업된 costmap과 병합하여 사용하려먼 다음과 같이 costmap을 가져와 사용하면 된다. 이 경우에 이전 layer에서 계산된 정보를 가져올 수 있다.
>
> ```cpp
> nav2_costmap_2d::Costmap2D * costmap = layered_costmap_->getCostmap();
> ```

`updateCosts()`에서 cost를 부여하는 방식을 결정하는 주요 로직이 들어가게 된다. 이 함수 내에 사용자가 의도한 cost 계산 방법을 구현한다.

```cpp
  /**
   * @brief Actually update the underlying costmap, only within the bounds
   *        calculated during UpdateBounds().
   */
  virtual void updateCosts(
    Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) = 0;
```

### 1.2.4 matchSize()

`matchSize()`는 맵 사이즈가 변경될 때마다 호출된다.

```cpp
  /** @brief Implement this to make this layer match the size of the parent costmap. */
  virtual void matchSize() {}
```

### 1.2.5 onFootprintChanged()

`onFootprintChanged()`는 footprint가 변경될 때마다 호출된다. footprint가 변경되었다는 것은 로봇의 위치가 변경되었다는 의미이므로 `need_recalculation_`을 true로 업데이트하고, cost 계산 영역을 업데이트하도록 트리거한다.

```cpp
  /** @brief LayeredCostmap calls this whenever the footprint there
   * changes (via LayeredCostmap::setFootprint()).  Override to be
  * notified of changes to the robot's footprint. */
  virtual void onFootprintChanged() {}
```

### 1.2.6 reset()

`reset()`는 costmap을 리셋하는 경우 사용된다.

```cpp
  /**
   * @brief Reset this costmap
   */
  virtual void reset() = 0;
```

## 2. Costmap2D 플러그인 Export 하기

작성된 Costmap2D 플러그인은 기본 부모 클래스로 런타임에 로드된 후 플러그인 처리 모듈에 의해 호출된다. Pluginlib는 런타임에 지정된 플러그인을 열고 내보낸 클래스에서 호출할 수 있는 메서드를 제공하는데, 클래스 내보내기 메커니즘은 이러한 호출 중에 사용해야 하는 기본 클래스를 pluginlib에 알려준다. 이를 통해 애플리케이션 소스 코드를 모르거나 다시 컴파일하지 않고도 플러그인으로 애플리케이션을 확장할 수 있다.

### 2.1 PLUGINLIB_EXPORT_CLASS

플러그인 클래스가 작성된 `.cpp` 파일 끝에 `PLUGINLIB_EXPORT_CLASS` 매크로를 추가한다.

```cpp
// This is the macro allowing a nav2_gradient_costmap_plugin::GradientLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_gradient_costmap_plugin::GradientLayer, nav2_costmap_2d::Layer)
```

### 2.2 플러그인 description 파일

`.xml` 파일을 만들어 플러그인 정보를 담는다.

- `path`: 플러그인이 배치되는 라이브러리의 경로 및 이름
- `name`: `plugin_types` 파라미터에 사용될 에서 참조되는 플러그인 타입. 즉, 플러그인을 참조할 때 사용할 이름을 뜻한다.
- `type`: 소스 코드에서 가져온 네임스페이스를 포함하는 플러그인 클래스입니다.
- `basic_class_type`: 플러그인 클래스가 파생된 기본 부모 클래스입니다. 일반적으로 Costmap2D 플러그인은 `nav2_costmap_2d::Layer`가 될 것이다.
- `description`: 플러그인에 대한 간략한 설명

```xml
<library path="nav2_gradient_costmap_plugin_core">
  <class name="nav2_gradient_costmap_plugin/GradientLayer"
  type="nav2_gradient_costmap_plugin::GradientLayer"
  base_class_type="nav2_costmap_2d::Layer">
    <description>This is an example plugin which puts repeating costs gradients to costmap</description>
  </class>
</library>
```

### 2.3 CMake function

`CMakeLists.txt`에 `pluginlib_export_plugin_description_file()` 함수를 추가한다. 해당 함수에 앞에서 만든 description 파일을 `share` 디렉토리에 설치하고, ament 인덱스를 설정하여, 설정된 타입의 플러그인을 찾을 수 있도록 한다.

```cmake
pluginlib_export_plugin_description_file(nav2_costmap_2d gradient_layer.xml)
```

### 2.4 설정 파일

`package.xml`에 description 파일을 추가한다.

```xml
<export>
  <costmap_2d plugin="${prefix}/gradient_layer.xml" />
  <build_type>ament_cmake</build_type>
</export>
```

### 2.5 빌드

작업이 완료된 플러그인 패키지를 사용할 워크스페이스에서 빌드하면 된다.

## 3. 작성된 Costmap2D 플러그인 적용

만들어진 Costmap2D Layer를 적용하기 위해 네비게이션 parameter 파일에 추가한다. Nav2의 parameter는 `.yaml`에 정의되어있는데, global_costmap 또는 local_costmap에 `plugins` 리스트에 이름을 등록하고, 해당 이름에 해당하는 `plugin` 인덱스와 설정할 parameter 값을 작성하면 navigation 실행 시 적용된다.

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      # ...
      # global_costmap 파라미터
      # ...
      plugins: ["obstacle_layer", "gradient_layer"]
      obstacle_layer:
        # obstacle_layer 정의
      gradient_layer:
        plugin: nav2_gradient_costmap_plugin/GradientLayer
        enabled: true
      # ...
```

> `plugins`에 적용된 플러그인의 순서대로 실행되므로, layer 간의 순서도 고려해야 한다.

## 참고문헌

- [Writing a New Costmap2D Plugin](https://navigation.ros.org/plugin_tutorials/docs/writing_new_costmap2d_plugin.html)
- [[ROSCon 2014] David V. Lu!!: Navigation illumination: Shedding light on the ROS navstack](https://vimeo.com/106994708)
- [nav2_gradient_costmap_plugin](https://github.com/ros-planning/navigation2_tutorials/tree/master/nav2_gradient_costmap_plugin)
