---
layout: note_page
title: FlexBE 튜토리얼
tags: [FlexBE]
category: "Robotics"
---

튜토리얼에서는 실행 시 일정 시간이 지난 후에 ‘Hello World!’를 출력하는 behavior를 만드는 것을 목표로 한다.

## 1. Creating a New Behavior

다음의 roslaunch 명령을 통해 flexbe_app을 실행시킨다.

```bash
roslaunch flexbe_app flexbe_ocs.launch
```

실행 후 Behavior Deshboard가 보이는데, 다음과 같이 작성한다(각 항목의 세부 설정 사항은 [위키 사이트](http://wiki.ros.org/flexbe/Tutorials/Creating%20a%20New%20Behavior) 참조).

<img src="/assets/img/posts/240202_flexbe_dashboard.png">

- Overview는 해당 behavior의 이름과 그에 대한 개요를 작성한다.
    - 이 때 이름은 위 예제처럼 하지 않고, 공백 없이 소문자로 작성하는 것이 좋은 것 같다.
- Private Configuration 패널은 behavior 동작에 필요한 상수값을 정의한다.
- Behavior Parameters 패널은 behavior에 사용되는 매개 변수를 정의한다.
- State Machine Userdata
- State Machine Interface

## 2. Using the Statemachine Editor

다음으로 Statemachine Editor로 이동하여 동작에 필요한 state를 정의한다(state의 세부 생성 방법은 [위키 사이트](http://wiki.ros.org/flexbe/Tutorials/Using%20the%20Statemachine%20Editor) 참조). 여기서 주의할 점은 새로운 state를 만드는 것이 아니라 flexbe에서 제공하는 state를 이용하여 정의하는 것이다.

<img src="/assets/img/posts/240202_flexbe_state_editor.png">

- Initial_Wait는 WaitState state를 이용하여 정의되었고, WaitState에서 정의되어야 하는 파라미터는 wait_time이다.
    - wait_time을 앞서 Behavior Parameters에서 정의한 매개변수인 waiting_time으로 지정하였다.
    - Initial_Wait는 waiting_time의 값 만큼 대기하는 state이다.
    - 앞서 정의한 파라미터들은 self. 로 시작해야 하며, 이는 파라미터와 상수를 구분하기 위해 사용된다.
- Print_Gretting은 LogState state를 이용하여 정의되었고, LogState에서 정의되어야 하는 파라미터는 text와 serverity이다.
    - text는 앞서 정의한 상수인 hello로 정의하였고, serverity는 디폴트로 정의되어 있는 Logger.REPORT_HINT를 사용한다.
    - Required Autonomy Levels는 High로 지정한다.
    - Print_Gretting은 ‘Hello World!’를 출력하는 state이다.
    - 여기서 상수는 파라미터와 다르게 self.를 붙지이 않은 것을 알 수 있다.
- 시작점부터 각 state를 연결하고, 마지막엔 finished에 연결한다. 여기서 failed는 사용하지 않았다.
- [Save Behavior] 버튼을 통해 만든 behavior를 저장할 수 있고, 해당 behavior는 이전에 만든 repository에 생성됨을 확인할 수 있다.

## 3. Execution of a Behavior

Behavior를 만든 이후, 해당 behavior를 동작하기 위해 다음의 명령으로 flexbe의 전체 behavior engine을 실행할 수 있다.

```bash
roslaunch flexbe_app flexbe_full.launch
```

실행시키면 동일한 화면이 나오고, [Load Behavior] 버튼을 통해 위해서 만든 behavior를 가져올 수 있다.

<img src="/assets/img/posts/240202_flexbe_load.png">

RuntimeControl 탭으로 넘어가면 앞서 만든 behavior를 실행시켜 볼 수 있다.(자세한 실행 내용은 위키 페이지 참조)

<img src="/assets/img/posts/240202_flexbe_runtime_control.png">

여기서 Autonomy Level의 개념을 정의하는데,
- Autonomy Level은 각 state의 수행 수준을 정의하는데 사용된다.
- 실행 수준은 No, Low, High, Full의 네 가지 수준으로, 해당 수준의 이상인 state는 사용자가 직접 화살표를 클릭하여 진행시켜야 하며, 해당 수준 이하인 state는 자동적으로 진행이된다.
    - No인 경우에는 모든 state를 사용자가 직접 수행시키며, Low와 High는 각각 state의 Autonomy Level이 Low 또는 High 이상인 경우만 사용자가 직접 수행시키고, Full의 경우 모든 state가 자동으로 진행된다.


## 참고문헌

- [flexbe/Tutorials/Creating a New Behavior - ROS Wiki](http://wiki.ros.org/flexbe/Tutorials/Creating%20a%20New%20Behavior)
- [flexbe/Tutorials/Using the Statemachine Editor - ROS Wiki](http://wiki.ros.org/flexbe/Tutorials/Using%20the%20Statemachine%20Editor)
- [flexbe/Tutorials/Execution of a Behavior - ROS Wiki](http://wiki.ros.org/flexbe/Tutorials/Execution%20of%20a%20Behavior)