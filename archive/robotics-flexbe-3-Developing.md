---
layout: note_page
title: FlexBe - Developing Basic States
tags: [FlexBE]
category: "Robotics"
---

# 1. State

States는 Python으로 적상되며, state의 클래스는 EventState 클래스를 상속받아 만든다.

```python
from flexbe_core import EventState, Logger
```

> 여기서 Logger는 FlexBE의 log 메세지를 나타내기 위해 사용된다.

### Constructor

EventState 클래스의 생성자는 다음과 같다

```python
def __init__(self, target_time):
        super(ExampleState, self).__init__(outcomes = ['continue', 'failed'])

        self._target_time = rospy.Duration(target_time)
        self._start_time = None
```

생성자를 통해서 변수을 선언하고, proxy를 초기화 한다. 특히 생성자의 superclass의 생성자를 호출해야 하는데, superclass는 다음의 세 가지의 인수를 사용한다.

- outcomes(required) : 해당 state가 리턴하는 값들의 리스트로, 해당 값에 따라 다음 state를 선택할 수 있다.
- input_keys(optional) : 해당 state에 필요한 userdata의 리스트로, 이전 state로부터 받은 input_keys 값들을  userdata에 선언되어 있어 접근할 수 있다.
- output_keys(optional) : 해당 state에 의해 제공되는 userdata의 리스트로, 다음 state가 해당 state에서 생성된 output_keys값들을 받을 수 있다.

### Execution Loop

```python
def execute(self, userdata):
        if rospy.Time.now() - self._start_time < self._target_time:
                return 'continue'
```

Execution loop는 execute 함수에 의해 실행되며, state가 활성 상태에 있는 동안에 주기적으로 실행된다(default: 10hz). execute함수는 outcomes에 해당하는 리스트 중 어떤 값을 리턴할 지 정하게 되는데, 예를 들어 위의 예에서 조건이 충족되지 않아 리턴이 되지 않는 경우, 계속 활성 상태에 있게 된다.

또한 execute함수는 userdata를 인자로 받는데, 이 변수는 생성자에 정의된 input/output key에 접근할 수 있도록 한다.

* input key는 read만 할 수있고, output key는 write만 할 수 있음.

```python
my_value = userdata.my_defined_input_key
userdata.my_defined_output_key = my_value
```

### Event Callbacks

* [Events 정의 사이트]()
* 특정 이벤트를 호출하기 위한 몇 가지 추가 함수를 제공한다

#### on_start

```python
def on_start(self):
```

on_start 이벤트는 동작이 시작될 때 트리거 되나, 일반적으로 생성자가 해당 역할을 하므로 해당 이벤트는 잘 사용되지 않는다. 만약 behavior가 실패하는 경우 해당 이벤트는 동작하지 않기 때문에 생성자를 사용하는 것이 더 좋다.

#### on_enter

```python
def on_enter(self, userdata):
```

on_enter 이벤트는 state가 활성 상태로 활성화 될 때 한 번 호출되며, 일반적으로 변수를 초기 값으로 설정하거나 action goal을 보낼 때 사용된다.

#### on_exit

```python
def on_exit(self, userdata):
```

on_exit 이벤트는 결과가 반환되고 다른 state가 활성화될 때 트리거된다. on_enter에 의해 시작된 실행 프로세스를 중지하는 데 사용할 수 있다.

#### on_stop

```python
def on_stop(self):
```

on_stop 이벤트는 behavior가 중지 또는 취소 될 때 트리거된다. 이 이벤트를 사용하여 state가 활성되지 않더라도 요청되거나 실행중인 state를 정리한다. 만약 proxy를 사용한다면, 이를 해제할 필요가 없다.

#### on_pause

```python
def on_pause(self):
```

on_pause 이벤트는 외부 인터페이스에서 pause 명령이 실행될 때 트리거된다. 이 상태에서 이루어지는 작업이 로봇에 영향이 생기지 않도록 이벤트를 구현해야 한다. state가 일시 중단된 동안에는 execute loop가 호출되지 않는다. 이이벤트는 on_exit과 유사하게 이벤트를 구현할 수 있다.


#### on_resume

```python
def on_resume(self, userdata):
```

on_resume 이벤트는 중지 이후 on_pause 이벤트를 벗어나 execute loop를 다시 실행할 때 트리거된다. 이 이벤트는 on_enter과 유사하게 이벤트를 구현할 수 있다.

# 2. Proxies

state에서 topic을 pub/sub 하거나, service 또는 action 메세지를 사용을 하게 되는데, 이를 효율적으로 사용하기 위해 FlexBE에서 다음과 같은 proxies를 제공한다.

```python
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyServiceCaller
from flexbe_core.proxy import ProxyActionClient

# Publisher
self._pub = ProxyPublisher({'/publisher/topic' : TopicMsgType})
# Examples
self._pub.publish('/publisher/topic', TopicMsgs)

# Subscriber
self._sub = ProxySubscriberCached({'/subscriber/topic': TopicMsgType})
# Examples
if self._sub.has_msg('/subscriber/topic'):
    TopicMsgs = self._sub.get_last_msg('/subscriber/topic')
    self._sub.remove_last_msg('/subscriber/topic')

# Service Client
self._srv_client = ProxyServiceCaller({'/service/topic' : ServiceMsgType})
# Examples
result = self._srv_client.call('/service/topic', ServiceMsgTypeRequest())

# Action Client
self._action_client = ProxyActionClient({'/action/topic' : ActionMsgType})
# Examples
self._action_client.send_goal('/action/topic', ActionGoal)

if self._action_client.has_result('/action/topic'):
    result = self._action_client.get_result('/action/topic')
    
if self._action_client.has_feedback('/action/topic'):
    feedback = self._action_client.get_feedback('/action/topic')
    self._action_client.remove_feedback('/action/topic')
```

# 참고문헌

- [flexbe/Tutorials/Developing Basic States - ROS Wiki](http://wiki.ros.org/flexbe/Tutorials/Developing%20Basic%20States)
- [flexbe/Tutorials/The State Lifecycle - ROS Wiki](http://wiki.ros.org/flexbe/Tutorials/The%20State%20Lifecycle)