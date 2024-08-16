---
layout: note_page
title: ROS2 Executors
tags: [ROS2]
category: "Robotics"
---

ROS2 노드의 실행 관리를 수행하는 Executors에 대해서 알아보고, Executors를 이용한 멀티 스레드의 운용, ROS 인터페이스의 콜백 관리 등에 대해서 정리한다.

# 1. Executor를 이용한 노드 spin

ROS2 C++ 패키지에서 rclcpp Node를 생성하고, 생성된 노드를 실행하기 위하여 일반적으로 `rclcpp::spin()` 메소드를 사용한다. 사용 예는 아래와 같다.

```cpp
int main(int argc, char* argv[])
{
  // Some initialization.
  rclcpp::init(argc, argv);
  ...

  // Instantiate a node.
  rclcpp::Node::SharedPtr node = ...

  // Run the executor.
  rclcpp::spin(node);

  // Shutdown and exit.
  ...
  return 0;
}
```

위에서 `node` 인스턴스를 생성해서 `spin()`을 함으로써 하나의 단일 노드가 실행이 되었는데, 이를 `executor`를 이용하면 다음과 같이 구현할 수 있다.

```cpp
int main(int argc, char* argv[])
{
  // Some initialization.
  rclcpp::init(argc, argv);
  ...

  // Instantiate a node.
  rclcpp::Node::SharedPtr node = ...

  // Run the executor.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  // Shutdown and exit.
  ...
  return 0;
}
```

`SingleThreadedExeuctor`를 이용하여 `node`를 `add_node()`를 이용하여 하나의 노드를 executor에 추가하고 이를 `spin()`한 것으로, 이는 executor를 이용한 가장 간단한 실행 방법이다.

> `spin()` 메소드는 executor에 추가된 노드에 대하여 스레드를 동작시키며, 노드가 shutdown 될 때까지 노드 내에 정의된 콜백 함수들을 호출시키는 역할을 한다.

# 2. Executor의 유형

executor에는 세 가지 유형을 가진다.

<img src="/assets/img/posts/230213_executors_type.png">

`SingleThreadedExecutor`는 가장 기본적인 executor로, 이름 그대로 스레드가 하나인 노드로 생성이 되기 때문에, 모든 ROS 인터페이스를 하나의 스레드에서 처리하기 때문에, 하나의 ROS 인터페이스 콜백에서 block이 되면 해당 노드가 멈추게 될 수도 있다.

반면에 `MultiThreadedExecutor`는 스레드를 병렬로 처리할 수 있게 여러 개의 스레드를 만들 수 있는 노드로 생성을 시킨다. 이는 `SingleThreadedExecutor`와 다르게 `Callback Groups`을 지정하면 그 그룹에 따라 ROS 인터페이스를 병렬로 처리하기 때문에, 그룹 내에서(즉, 하나의 스레드 안에서) 발생한 block이 다른 그룹에 영향을 주지 않는다.

`StaticSingleThreadedExecutor`는 한 개의 스레드에서 subscription이나 timer, service나 action server의 callback에 대하여 노드 구조를 스캔하기 위한 런타임 비용을 최적화 한다고 한다. 즉, 해당 실행자는 노드가 추가될 때 정의되는 subscription, timer, service나 action server에 대해서만 싱글 스레드에 최적화를 위한 스캔을 수행하고, 그 이후에 런타임에 생성되는 인터페이스에 대해서는 최적화를 수행하지 않는다. 따라서 해당 생성자는 초기화 이후에 인터페이스를 생성하는 노드에서는 사용해서는 안된다.

> `StaticSingleThreadedExecutor`에 대해서는 더 연구가 필요하다.

# 3. Callback Groups

콜백 그룹은 `MultiThreadedExecutor`로 추가된 노드에서 subscription, timer, service server, action server에서 호출되는 callback에 대한 단위 그룹을 지정할 수 있도록 지정하는 그룹이다. 위에서 설명했듯이, callback 그룹 내에서 특정 지점에서 block이 발생할 경우, 해당 callback 그룹은 전체가 block이 되어 기능을 수행하지 못할 수 있다. 따라서 각 기능별로 callback 그룹을 적절히 지정하는 것이 중요하다.

callback group의 선언 방법은 다음과 같다.

```cpp
my_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

rclcpp::SubscriptionOptions options;
options.callback_group = my_callback_group;

my_subscription = create_subscription<Int32>("/topic", rclcpp::SensorDataQoS(),
                                             callback, options);
```

> callback group 인자 생성 시, 노드 전체에 걸쳐 저장될 수 있도록, 클래스 멤버 변수 또는 전역 변수로 선언되어야 한다. 그렇지 않고, 스택 영역에 저장되는 경우 스택 영역을 벗어나게 되면 `nullptr`이 되어 같은 그룹으로 지정이 되게 된다.

callback group에는 다음과 같이 두 가지 타입이 존재한다.

- `MutuallyExclusive`: 상호 배타적이라는 의미로, 해당 타입으로 선언된 콜백 그룹을 공유하는 인자들은 병렬적으로 처리될 수 없다. 즉, 같은 mutually 그룹은 하나로 병렬로 처리되지 못한다는 것이고, 단, 다른 group과는 배타적으로 실행되는 그룹이다.
- `Reentrant`: 해당 타입으로 선언된 콜백 그룹을 공유하는 인자들은 서로 병렬적으로 처리된다. 즉, 같은 reentrant 그룹은 서로가 병렬적으로 처리된다는 것이다. 이 그룹 또한 다른 group과는 배타적으로 실행된다.

콜백 그룹을 적절히 지정하는 것은 매우 중요하다. 무조건 병렬적으로 처리해야 되는 그룹에 대해서 `Reentrant` 그룹으로 하나로 묶어서 병렬로 처리를 할 수도 있고, `MutuallyExclusive` 그릅으로 두 개로 나눠서 배타적 처리를 할 수도 있다. 방법은 여러가지일 수 있으나, 병렬로 처리해야하는데, 디폴트 콜백그룹을 사용하여, 병렬처리를 하지 못하는 상황을 만들어서는 안된다.

# 참고문헌

- [ROS 2.0 humble Executors Documentation](https://docs.ros.org/en/humble/Concepts/About-Executors.html)
