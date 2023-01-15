---
layout: post
title: ROS2 C++ 기본 디자인 패턴
feature-img: "assets/img/posts/ros.png"
thumbnail: "assets/img/posts/ros.png"
tags: [ROS2]
---

ROS2 패키지 작성 시 패키지의 구조와 인터페이스 정의 및 구현 시 사용되는 기본 패턴을 정리한다.

## 1. ROS2 C++ 패키지 구조

해당 구조는 예시 패키지로 `ros2_design_example`이라는 패키지의 예시 구조이다.

```bash
ros2_design_example
├── include
│   └── ros2_design_example
│       ├── submodule1
│       │   ├── submodule1_class1.hpp
│       │   └── submodule1_class2.hpp
│       ├── submodule2
│       │   ├── submodule2_class1.hpp
│       │   └── submodule2_class2.hpp
│       └── ros2_design_example.hpp
│       └── ros2_design_example2.hpp
├── launch
│   └── ros2_design_example.launch.py
├── src
│   ├── submodule1
│   │   ├── submodule1_class1.cpp
│   │   └── submodule1_class2.cpp
│   ├── submodule2
│   │   ├── submodule2_class1.cpp
│   │   └── submodule2_class2.cpp
│   ├── ros2_design_example.cpp
│   ├── ros2_design_example2.cpp
│   └── main.cpp
├── CMakeLists.txt
├── package.xml
└── README.md
```

- `ros2_design_example`이라는 패키지를 만들었을 때, 가장 기본적인 `CMakeLists.txt`, `package.xml`와 패키지에 대한 설명을 담은 `README.md`가 있고, 소스 코드인 `src` 폴더와 헤더가 정의된 `include/ros2_design_example` 폴더, `launch` 파일이 저장된 `launch` 폴더를 갖는다.
- 그 이외에 패키지의 속성에 따라 `map`, `param`, `world`, `rviz` 등 속성에 맞는 폴더를 만들고, 해당 폴더 내에 패키지 실행 시 필요한 데이터들을 저장한다.
- `include`와 `src`에서는 해당 패키지에서 사용할 모듈들을 구분하여 저장하고, 모듈에 필요한 클래스들을 구현한다.
- `main.cpp`는 `ros2_design_exmaple.cpp`에 정의된 노드 클래스를 가져와 실행한다.
- `ros2_design_example.cpp`는 메인 노드 클래스를 정의하고, 인터페이스를 선언하며, 정의된 `submodule`들을 이용하여 로직을 구현한다.

## 2. ROS2 C++ 디자인 패턴

- ROS2의 Executor에 대한 내용은 [ROS2 Documentation Executors](https://docs.ros.org/en/foxy/Concepts/About-Executors.html)를 참조한다.

### 2.1 main.cpp에서의 노드 생성

#### 싱글 노드인 경우

`rclcpp::executors::SingleThreadedExecutor`를 이용한다.

```cpp
#include <ros2_design_example/ros2_design_example.hpp>

int main(int argc, char ** argv)
{
  std::cout << "ROS2 Design Example ROS2 Node Started..." << std::endl;

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  auto example = std::make_shared<DesignExample>();
  executor.add_node(example);

  executor.spin();
  rclcpp::shutdown();

  return 0;
}
```

#### 멀티 노드인 경우

`rclcpp::executors::MultiThreadedExecutor`를 이용하여 여러 개의 노드를 한번에 실행한다.

```cpp
#include <ros2_design_example/ros2_design_example.hpp>
#include <ros2_design_example/ros2_design_example2.hpp>

int main(int argc, char ** argv)
{
  std::cout << "ROS2 Design Example ROS2 Node Started..." << std::endl;

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto example = std::make_shared<DesignExample>();
  auto example2 = std::make_shared<DesignExample2>();
  executor.add_node(example);
  executor.add_node(example2);

  executor.spin();
  rclcpp::shutdown();

  return 0;
}
```

#### 멀티 노드를 동적으로 생성하는 경우

`executor`를 `MultiThreadedExecutor`의 `shared_ptr`로 생성하여 이를 노드 생성 시 공유 인자로 넘기면, 해당 노드 안에서 `executor`를 이용하여 `add_node`, `remove_node` 등을 수행할 수 있다.

```cpp
#include <ros2_design_example/ros2_design_example.hpp>

int main(int argc, char ** argv)
{
  std::cout << "ROS2 Design Example ROS2 Node Started..." << std::endl;

  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  auto example = std::make_shared<DesignExample>(executor);
  executor.add_node(example);

  executor.spin();
  rclcpp::shutdown();

  return 0;
}
```

### 2.2 ros2_design_example 코드

#### ros2_design_example.hpp

```cpp
#ifndef ROS2_DESIGN_EXAMPLE__ROS2_DESIGN_EXAMPLE_HPP_
#define ROS2_DESIGN_EXAMPLE__ROS2_DESIGN_EXAMPLE_HPP_

// C++ 라이브러리 include
#include <chrono>

// rclcpp include
#include <rclcpp/rclcpp>

// ROS 인터페이스 패키지 include
#include <example_msgs/msg/example_pub.hpp>
#include <example_msgs/msg/example_sub.hpp>
#include <example_msgs/srv/example_client.hpp>
#include <example_msgs/srv/example_server.hpp>

// 패키지 내부 라이브러리 include
#include <ros2_design_example/submodule1/submodule1_class1.hpp>
#include <ros2_design_example/submodule1/submodule1_class2.hpp>
#include <ros2_design_example/submodule2/submodule2_class1.hpp>
#include <ros2_design_example/submodule2/submodule2_class2.hpp>

// 패키지 외부 라이브러리 include

namespace ros2_design_example
{

class DesignExample : public rclcpp::Node
{
// class 구현
public:
  using ExamplePub = example_msgs::msg::ExamplePub;
  using ExampleSub = example_msgs::msg::ExampleSub;
  using ExampleClient = example_msgs::srv::ExampleClient;
  using ExampleServer = example_msgs::srv::ExampleServer;
  
  DesignExample();
  ~DesignExample();
  
private:
  // ROS2 인터페이스
  void example_pub();
  rclcpp::Publisher<ExamplePub>::SharedPtr example_pub_;
  
  void example_sub_callback(ExampleSub::UniquePtr msg);
  rclcpp::Subscription<ExampleSub>::SharedPtr example_sub_;
  
  void request_example_client();
  rclcpp::Client<ExampleClient>::SharedPtr example_cli_;
  
  void example_server_callback(
    const std::shared_ptr<ExampleServer::Request> request,
    const std::shared_ptr<ExampleServer::Response> response);
  rclcpp::Service<ExampleServer>::SharedPtr example_srv_;
  
  // thread
  void example_thread(unsigned int ms);
  std::shared_ptr<std::thread> example_thread_;
  std::mutex example_mutex_;
};

}

#endif // ROS2_DESIGN_EXAMPLE__ROS2_DESIGN_EXAMPLE_HPP_
```

#### ros2_design_example.cpp

```cpp
#include <ros2_design_example/ros2_design_example.hpp>

namespace ros2_design_example
{

DesignExample::DesignExample()
: Node("Design_Example_Node")
{
  // ROS2 인터페이스 초기화
  // publisher
  auto example_pub_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  example_pub_ =
    create_publishe<ExamplePub>("/example/pub", example_pub_qos)
    
  // subscription
  auto example_sub_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  example_sub_ = create_subscription<ExampleSub>(
    "/example/sub", example_sub_qos,
    [ = ](ExampleSub::UniquePtr msg)
    {
      example_sub_callback(std::move(msg));
    },
    rclcpp::Subscriptions());
    
  // Service Client
  example_cli_ =
    create_client<ExampleClient>("/example/client");
    
  // Service Server
  example_srv_ = create_service<ExampleServer>(
    "/example/server",
    [ = ](
      const std::shared_ptr<ExampleServer::Request> request,
      const std::shared_ptr<ExampleServer::Response> response)
    {
      example_server_callback(request, response);
    });
  
  // thread 설정
  const unsigned int thread_ms = 1000;
  example_thread_ = 
    std::make_shared<std::thread>(
    std::bind(&DesignExample::example_thread, this, thread_ms));
}

DesignExample::~DesignExample()
{
  example_thread_->detach();
  if(example_thread_->joinable()) {example_thread_->join();}
}

// Publish 구현 =====================================================
void DesignExample::example_pub()
{
  auto msg = std::make_unique<ExamplePub>();
  
  // msg-> 으로 publish할 message에 저장
  
  example_pub_->publish(std::move(msg));
}

// Subscription 구현 ================================================
void DesignExample::example_sub_callback(ExampleSub::UniquePtr msg)
{
  // msg-> 으로 publish 된 message에 접근
}

// Service Client 구현 ==============================================
void DesignExample::request_example_client()
{
  auto req = std::make_shared<ExampleClient::Request>();
  // req-> 으로 request에 저장
  
  using namespace std::chrono_literals;
  while(!example_cli_->wait_for_service(1s)){
    if(!rclcpp::ok()){
      RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"),
        "[ExampleClient]Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp"),
      "[RequestRobotState]service not available, waiting again...");
    return;
  }
  
  auto tp = std::chrono::system_clock::now() + std::chrono::seconds(1);
  auto res = example_cli_->async_send_request(req).get();
  auto status = res.wait_until(tp);
  
  if(status == std::future_status::ready){
    // res-> 으로 response에 접근
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("rclcpp"), "[ExampleClient]Timeout");
    return;
  }
}

// Service Server 구현 ==============================================
void DesignExample::example_server_callback(
  const std::shared_ptr<ExampleServer::Request> request,
  const std::shared_ptr<ExampleServer::Response> response)
{
  // request-> 으로 request에 접근
  
  // request 처리
  
  // response-> 으로 response에 저장
}

void DesignExample::example_thread(unsigned int ms)
{
  const auto wait_duration = std::chrono::milliseconds(ms);
  while (true) {
    // sleep_for을 맨 처음 또는 맨 끝에서 실행
    std::this_thread::sleep_for(wait_duration);
    
    // example_mutex_를 이용하여 lock, unlock
    std::unique_lock<std::mutex> example_lock(example_mutex_);
    // ... //
    example_lock.unlock();
  }
}

}
```

### 2.3 executor를 이용하여 ros2_design_example 노드 내에서 새로운 노드를 추가하는 경우

#### 외부 노드

외부 노드에서는 `main.cpp`에서 생성된 `executor`를 `shared_ptr`로 공유받아 같은 `executor`를 사용하는 것이 핵심이며, 이 `executor`를 이용하여 소스코드 내에서 `add_node`, `remove_node`를 수행할 수 있다.

```cpp
// ros2_design_example.hpp

#ifndef ROS2_DESIGN_EXAMPLE__ROS2_DESIGN_EXAMPLE_HPP_
#define ROS2_DESIGN_EXAMPLE__ROS2_DESIGN_EXAMPLE_HPP_

// rclcpp include
#include <rclcpp/rclcpp>
#include <ros2_design_example/ros2_design_example2.hpp>

namespace ros2_design_example
{

class DesignExample : public rclcpp::Node
{
// class 구현
public:  
  DesignExample(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor);
  ~DesignExample();
  
private:
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::map<std::string, std::shared_ptr<DesignExample2> inner_node_set_;
};

}

#endif // ROS2_DESIGN_EXAMPLE__ROS2_DESIGN_EXAMPLE_HPP_
```

```cpp
// ros2_design_example.cpp
#include <ros2_design_example/ros2_design_example.hpp>

namespace ros2_design_example
{

DesignExample::DesignExample(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor)
: Node("Design_Example_Node"), executor_(executor)
{

}

DesignExample::~DesignExample()
{

}

void DesignExample::generate_node()
{
  std::string node_id = generate_node_id();
  std::shared_ptr<DesignExample2> inner_node = std::make_shared<DesignExample2>(node_id);
  executor_->add_node(inner_node);
  inner_node_set_.insert(std::make_pair(node_id, inner_node));
}

void DesignExample::delete_node(std::string node_id)
{
  auto it = inner_node_set_.find(node_id);
  if(it != inner_node_set_.end()){
    executor_->remove_node(it->second);
    inner_node_set_.erase(it->first);
  }
}

}
```

#### 내부 노드

내부 노드를 생성시 해당 노드도 `rclcpp::Node`를 상속받은 노드로 구현되어야 한다.

```cpp
// ros2_design_example2.hpp

#ifndef ROS2_DESIGN_EXAMPLE__ROS2_DESIGN_EXAMPLE2_HPP_
#define ROS2_DESIGN_EXAMPLE__ROS2_DESIGN_EXAMPLE2_HPP_

// rclcpp include
#include <rclcpp/rclcpp>

namespace ros2_design_example
{

class DesignExample2 : public rclcpp::Node
{
// class 구현
public:  
  DesignExample2(std::string node_id);
  ~DesignExample2();
  
private:

};

}

#endif // ROS2_DESIGN_EXAMPLE__ROS2_DESIGN_EXAMPLE_HPP_
```