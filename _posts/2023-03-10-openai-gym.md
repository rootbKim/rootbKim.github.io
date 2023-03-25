---
layout: post
title: OpenAI Gym
feature-img: "assets/img/portfolio/alr_logo.png"
thumbnail: "assets/img/portfolio/alr_logo.png"
tags: [Gym Gazebo]
---

OpenAI에서 제공하는 GYM 라이브러리에 대해서 정리하고, gym 라이브러리를 사용하기 위한 기본 API 문서를 정리한다.

## 1. OpenAI gym

[Open AI Gym 소개](https://c548adc0c815.gitbooks.io/javacafe-ml/content/open-ai-gym.html) 포스팅에서 Open AI Gym에 대해서 간략하게 잘 정리해주고 있다. 포스팅 내용을 간략하게 정리하면, 강화학습 알고리즘을 비교해보고 테스트해 볼 수 있도록 돕는 도구로써 강화학습을 위한 환경을 제공해주는 역할을 한다.

## 2. OpenAI의 사용

해당 블로그의 [OpenAI GYM을 이용한 간단한 게임 실습](https://c548adc0c815.gitbooks.io/javacafe-ml/content/open-ai-gym-ex.html)에 Gym 라이브러리의 간단한 사용 예를 보여주고 있다.

gym 패키지를 import하고, Gym 라이브러리에서 기본으로 제공하는 환경 중 하나인 `FrozenLake-v0` 환경을 생성한다. 그리고, `observation = env.reset()`으로 환경을 초기화하고, 그 observation(상태)를 가지고 온다.

```python
import gym
env = gym.make("FrozenLake-v0")
observation = env.reset()
```

`env.render()` 메소드로 현재 환경을 출력하고, `action`은 환경에 따른 적절한 행동을 정하는 부분인데, `env.action_space.sample()`은 랜덤하게 action을 정하는 메소드이다. 그리고 이 action에 대하여 `step`을 진행하여 그 결과인 `observation`, `reward`, `done`, `info` 정보를 획득한다.

```python
for _ in range(1000):
    env.render()  # 출력
    action = env.action_space.sample()  # agent의 action
    observation, reward, done, info = env.step(action)
```

## 3. ENV

### 3.1 step()

`action`을 파라미터로 받아 그 액션을 수행한다.

- `observation`(Object): 현재 환경에 대한 상태 정보로, 현재 환경의 요소가 된다. 예를 들어, 특정 물체의 위치와 속도를 가지고 있는 배열일 수 있다.
- `reward`(float): 액션의 수행 결과로 반환되는 보상
- `done`(bool): episode가 종료 여부를 반환, `done`이 반환되면 그 이상의 `step()`은 undefined result를 반환한다. `done`이 반환되는 경우는 작업이 성공적으로 수행되었거나, 정해진 시간이 초과되었거나, 물리 환경이 유효하지 않은 상태가 되었을 때 반한된다.
- `info`(dictionary): 학습에 대한 구체적인 정보를 반환한다.

### 3.2 reset()

환경을 초기 상태로 재설정하고 초기 `observation`을 반환한다. 입력 파라미터로 `seed`를 받아 환경을 랜덤하게 설정할 수 있다.

### 3.3 render()

`render_mode`에 정의된 데로 render frame을 계산한다. 지원되는 모드는 환경 세팅에 따라 달라질 수 있고, 기본적으로 `render_mode`는 다음과 같다.

- `None`: default 값으로 랜더링 하지 않는다.
- `human`: 환경은 현재 디스플레이 또는 터미널에서 지속적으로 렌더링됩니다. 일반적으로 사람에 의해 소비된다?
- `rgb_array`: rgb차원으로 구성된 3차원 array가 리턴된다.
- `rgb_array_list`: rgb차원으로 구성된 3차원 array list가 리턴된다.
- `ansi`: `string` 또는 `StringIO`를 반환한다. `StringIO`는 각 타임 스탭에서의 터미널 상의 텍스트를 나타낸다. 

### 3.4 close()

`close` 함수를 이용하여 `Env`의 종료를 정의한다. 

### 3.5 action_space

`action_space`는 action의 유효 범위를 부여하며, Gym에서 제공되는 `Space`라는 데이터 타입이다. 예를 들어, `action_space`가 `Discrete` 타입이면, `Discrete(2)`로 선언하고, action이 0 또는 1이라는 것을 나타낸다.

### 3.6 observation_space

`observation_space`는 observation의 유효 범위를 부여하며, 마찬가지로 `Space` 데이터 타입이다. 예를 들어, `observation_space`가 `Box` 타입이면, 해당 object는 `(4,)` 형태이며, observation이 4개의 숫자의 배열이라는 것을 나타낸다.

### 3.7 reward_range

`reward_range`는 reward 값의 최소, 최대값에 대한 tuple로, 디폴트 값은 `(-inf,+inf)`이다.

## 4. 새로운 Env 만들기

[Make your own custom environment](https://www.gymlibrary.dev/content/environment_creation/)에 가면 새로운 Env 클래스를 생성하는 자세한 방법에 대해 나와있다.

### 4.1 GridWorldEnv

새로운 `GridWorldEnv` 예제의 디렉토리 구조는 다음과 같다.

```bash
gym-examples/
  README.md
  setup.py
  gym_examples/
    __init__.py
    envs/
      __init__.py
      grid_world.py
    wrappers/
      __init__.py
      relative_position.py
```

이 예제는 고정된 크기의 2차원 정사각형 그리드로 구성된다. agent는 각 타임 스탭에서 수직 또는 수평으로 이동할 수 있으며, agent의 목표는 episode의 시작에서 랜덤하게 위치해 있을 때 목적지까지 이동하는 것이다.

- `Observation`은 target과 agent의 위치를 나타낸다.
- `Action`은 총 4 가지로, `right`, `up`, `left`, `down`이다.
- `Done` 신호는 agnet가 목적지에 도착하면 발행된다.
- `Reward`는 바이너리이며, `sparse data(희소 데이터: 차원/전체 공간에 비해 데이터가 있는 공간이 매우 협소한 데이터)`이다. 즉각적인 보상은 0이고, 에이전트가 목적지에 도착하지 않으면 1이다.

### 4.2 선언 및 정의

`GridWorldEnv` 클래스는 추상 클래스인 `gym.Env` 클래스를 상속받는다. `metadata`를 선언하고, `render_modes`와 `render_fps`를 지정해야 한다. 만약 랜더링하지 않는다면, 즉 `None`이라면 metadata에 추가하지 않아도 된다.

클래스 생성 시 인자로 size를 받아 그리드의 사이즈를 결정하고, `observation_space`와 `acction_space`와 같은 변수를 정의한다. 그리고 학습에 필요한 변수들을 등록한다. 여기서 `observation`은 agent와 target의 위치를 반환해야 하므로, `observation_space`를 agnet와 target에 대한 dictionary로 정의하였다. 따라서 observation은 `{"agent": array([1, 0]), "target": array([0, 3])}`과 같은 형태가 될 것이다. `action_space`는 `(“right”, “up”, “left”, “down”)`로 타나태지는 4개의 action으로 정의되므로, `Discrete(4)`로 정의하였다.

```python
import gym
from gym import spaces
import pygame
import numpy as np


class GridWorldEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 4}

    def __init__(self, render_mode=None, size=5):
        self.size = size  # The size of the square grid
        self.window_size = 512  # The size of the PyGame window

        # Observations are dictionaries with the agent's and the target's location.
        # Each location is encoded as an element of {0, ..., `size`}^2, i.e. MultiDiscrete([size, size]).
        self.observation_space = spaces.Dict(
            {
                "agent": spaces.Box(0, size - 1, shape=(2,), dtype=int),
                "target": spaces.Box(0, size - 1, shape=(2,), dtype=int),
            }
        )

        # We have 4 actions, corresponding to "right", "up", "left", "down"
        self.action_space = spaces.Discrete(4)

        """
        The following dictionary maps abstract actions from `self.action_space` to 
        the direction we will walk in if that action is taken.
        I.e. 0 corresponds to "right", 1 to "up" etc.
        """
        self._action_to_direction = {
            0: np.array([1, 0]),
            1: np.array([0, 1]),
            2: np.array([-1, 0]),
            3: np.array([0, -1]),
        }

        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode

        """
        If human-rendering is used, `self.window` will be a reference
        to the window that we draw to. `self.clock` will be a clock that is used
        to ensure that the environment is rendered at the correct framerate in
        human-mode. They will remain `None` until human-mode is used for the
        first time.
        """
        self.window = None
        self.clock = None
```

### 4.3 Observation과 Environments 상태 설계

`reset()`과 `step()`에서 `dobservation`을 계산하기 위한 메소드 `_get_bos()`를 정의한다. 이 메소드는 환경 상태를 `observation` 자료형으로 반환해준다. 그러나 이는 필수적으로 선언되어야 하는 것은 아니며, `reset()`과 `step()`에서 별도로 구현해도 상관없다. 

```python
    def _get_obs(self):
        return {"agent": self._agent_location, "target": self._target_location}
```

`step()`과 `reset()`에서 반환되는 `info` 정보를 비슷하게 정의할 수 있다. 여기서는 맨하탄 거리를 반환해준다.

```python
    def _get_info(self):
        return {"distance": np.linalg.norm(self._agent_location - self._target_location, ord=1)}
```

### 4.4 reset()

`reset()` 메소드는 episode를 초기화 할 때 호출된다. `reset()` 메소드가 호출되기 전 까지 `step()` 메소드는 호출되지 않는다고 가정하며, 언제든지 호출될 수 있다. 특히 seeding할 때 자주 호출된다. 랜덤하게 환경을 초기화할 때 gym.Env 클래스의 메소드인 `self.np_random` 메소드를 사용하는 것이 추천된다. 이 예제에서는 agnet의 위치와 target 위치를 랜덤하게 설정하는데 사용된다.

`reset()`은 초기 `observation`과 부가 정보를 튜플로 리턴하며, 위에서 정의한 `_get_bos`와 `_get_info` 메소드를 사용하여 리턴한다.

```python
    def reset(self, seed=None, options=None):
        # We need the following line to seed self.np_random
        super().reset(seed=seed)

        # Choose the agent's location uniformly at random
        self._agent_location = self.np_random.integers(0, self.size, size=2, dtype=int)

        # We will sample the target's location randomly until it does not coincide with the agent's location
        self._target_location = self._agent_location
        while np.array_equal(self._target_location, self._agent_location):
            self._target_location = self.np_random.integers(
                0, self.size, size=2, dtype=int
            )

        observation = self._get_obs()
        info = self._get_info()

        if self.render_mode == "human":
            self._render_frame()

        return observation, info
```

### 4.5 step()

`step()` 메소드는 학습하고자 하는 환경의 핵심 로직이 담겨있는 함수이다. `action`을 받아서, 액션을 수행한 이후의 환경 상태를 계산하고, `4-tuple (observation, reward, done, info)`을 반환한다. 새로운 환경 상태를 한번 계산하면, 상태가 terminate 되었는지, 그에 따라 완료가 되었는지를 확인할 수 있다. 여기서는 `reward`로 `sparse binary`를 사용하기 때문에, 그 결과값 자체는 작을 것이다. 여기서도 `observation`과 `info`를 반환하기 위해 `_get_bos`와 `_get_info` 메소드를 사용하여 리턴하고 있다.

```python
    def step(self, action):
        # Map the action (element of {0,1,2,3}) to the direction we walk in
        direction = self._action_to_direction[action]
        # We use `np.clip` to make sure we don't leave the grid
        self._agent_location = np.clip(
            self._agent_location + direction, 0, self.size - 1
        )
        # An episode is done iff the agent has reached the target
        terminated = np.array_equal(self._agent_location, self._target_location)
        reward = 1 if terminated else 0  # Binary sparse rewards
        observation = self._get_obs()
        info = self._get_info()

        if self.render_mode == "human":
            self._render_frame()

        return observation, reward, terminated, False, info
```

### 4.6 rendering()

이 예제는 `PyGame`을 이용하여 랜더링한다. 이 예제는 `PyGame`을 이용하여 랜더링하는 경우 이 예제를 스켈레톤 버전으로 활용할 수 있다.

```python
    def render(self):
        if self.render_mode == "rgb_array":
            return self._render_frame()

    def _render_frame(self):
        if self.window is None and self.render_mode == "human":
            pygame.init()
            pygame.display.init()
            self.window = pygame.display.set_mode((self.window_size, self.window_size))
        if self.clock is None and self.render_mode == "human":
            self.clock = pygame.time.Clock()

        canvas = pygame.Surface((self.window_size, self.window_size))
        canvas.fill((255, 255, 255))
        pix_square_size = (
            self.window_size / self.size
        )  # The size of a single grid square in pixels

        # First we draw the target
        pygame.draw.rect(
            canvas,
            (255, 0, 0),
            pygame.Rect(
                pix_square_size * self._target_location,
                (pix_square_size, pix_square_size),
            ),
        )
        # Now we draw the agent
        pygame.draw.circle(
            canvas,
            (0, 0, 255),
            (self._agent_location + 0.5) * pix_square_size,
            pix_square_size / 3,
        )

        # Finally, add some gridlines
        for x in range(self.size + 1):
            pygame.draw.line(
                canvas,
                0,
                (0, pix_square_size * x),
                (self.window_size, pix_square_size * x),
                width=3,
            )
            pygame.draw.line(
                canvas,
                0,
                (pix_square_size * x, 0),
                (pix_square_size * x, self.window_size),
                width=3,
            )

        if self.render_mode == "human":
            # The following line copies our drawings from `canvas` to the visible window
            self.window.blit(canvas, canvas.get_rect())
            pygame.event.pump()
            pygame.display.update()

            # We need to ensure that human-rendering occurs at the predefined framerate.
            # The following line will automatically add a delay to keep the framerate stable.
            self.clock.tick(self.metadata["render_fps"])
        else:  # rgb_array
            return np.transpose(
                np.array(pygame.surfarray.pixels3d(canvas)), axes=(1, 0, 2)
            )
```

### 4.7 close()

`close()` 메소드는 환경에서 사용되고 있는 자원들을 종료시키며, 이 메소드는 반드시 사용되고 있는 자원들을 종료시키는 역할을 수행해야 한다.

```python
    def close(self):
        if self.window is not None:
            pygame.display.quit()
            pygame.quit()
```

### 4.8 ENV 등록

새로만들어진 `Env` 환경을 사용하기 위해서 아래와 같이 `register`를 이용하여 등록해야 한다. 이 코드는 `gym-examples/gym_examples/__init__.py`에 정의되어야 한다.

```python
from gym.envs.registration import register

register(
    id='gym_examples/GridWorld-v0',
    entry_point='gym_examples.envs:GridWorldEnv',
    max_episode_steps=300,
)
```
Environment ID는 세 가지로 구성되어 있는데, 네임스페이스(gym_examples)는 선택 사항이며, 클래스 이름(GirdWorld)은 필수적으로 포함되어야 한다. 그리고 버전정보(v0)는 선택사항이지만, 포함이 권장된다.

`max_episode_steps=300`은 `gym.make`에 의해 인스턴스화 되어 있으며, 현재 episode에서 `done` 신호가 target에 도착하거나 300 스텝에 도달한 경우에 발생하게 된다.

`max-episode_step`과 같이 다음과 같은 인자를 register에 추가할 수 있다.

<img src="/assets/img/posts/230310_gym_register.png">

최종적으로 위에서 만든 `gym_examples`를 import하고, `make()` 메소드를 이용하여 env를 등록하면 된다.

```python
import gym_examples
env = gym.make('gym_examples/GridWorld-v0')
```

만약 클래스의 입력 인자가 있다면 다음과 같이 `make()` 메소드를 사용하면 된다.

```python
env = gym.make('gym_examples/GridWorld-v0', size=10)
```

## 참고문헌

- [OpenAI gym github](https://github.com/openai/gym)
- [Gymnasium is a standard API for reinforcement learning, and a diverse collection of reference environments](https://gymnasium.farama.org/), 
- [Open AI Gym 소개](https://c548adc0c815.gitbooks.io/javacafe-ml/content/open-ai-gym.html)
- [OpenAI GYM을 이용한 간단한 게임 실습](https://c548adc0c815.gitbooks.io/javacafe-ml/content/open-ai-gym-ex.html)
- [Core gym.Env](https://www.gymlibrary.dev/api/core/)
- [Make your own custom environment](https://www.gymlibrary.dev/content/environment_creation/)