---
layout: archive
title: ros2learn을 이용한 gym-gazebo 학습
tags: [Gym Gazebo]
category: "Robotics"
---

gym-gazebo2를 이용해 학습을 수행하고, 학습된 결과를 이용하는 패키지인 ros2learn 패키지에 대해 간략하게 다룬다.

# 1. ros2learn 설치

ros2learn 설치는 결론부터 말하면 성공하지 못했다.

docker 이미지가 있지 않아서, 해당 환경으로 설치하는 것이 불가능하고, dockerfile을 이용한 설치도 제대로 설치되지 않는다. 아마도, 우분투 버전, python 버전, ROS 버전이 업그레이드 되면서 맞지 않는 것 같다.

아래의 내용은 ros2learn README를 기반으로 작성한 것으로 이후의 프로젝트에 참고하는 용도로 사용하기로 한다.

# 2. Train

`/experiments/examples/` 내에 예제 코드를 이용하여 학습시킬 수 있다. 학습에 대한 자세한 내용은 [ROS2Learn: a reinforcement learning framework for ROS 2](https://arxiv.org/pdf/1903.06282.pdf) 내용을 참조한다.

```bash
cd ~/ros2learn/experiments/examples/MARA
python3 train_ppo2_mlp.py -g
```

자신의 훈련된 신경망을 테스트하거나, gym-gazebo2에서 다른 환경으로 훈련하거나, hyperparameter를 테스트할 때 해당 알고리즘에서 사전에 값을 직접 업데이트해야한다.

# 3. Run a Trained Policy

앞에서 train한 결과를 확인하기 위해 `running-scripts` 중 하나를 사용할 수 있다. 아래의 예시와 같이 저장된 `ppo2_mlp`를 이용하여 실행시킬 수 있다.

먼저, `mara_mlp()` dictionary를 수정해야하기 위해 생성된 checkpoint를 `baselines/ppo2/defaults.py`에 학습된 결과를 수정한 후에 실행한다.

```bash
cd ~/ros2learn/experiments/examples/MARA
python3 run_ppo2_mlp.py -g -r -v 0.3
```

# 4. Visualize Traning Data

`logdir` 경로는 environment ID와 사용된 알고리즘에 따라 생성된다. `Tensorboard` 실행 및 웹 브라우저에 사용될 링크를 열어야 한다. 여기에 reward plot과 같은 유용한 그래프를 확인할 수 있다. 다른 경로에서 두 개 이상의 텐서보드 파일을 시각화하려는 경우에 대비하여 특정 포트 번호를 설정할 수도 있다.

```bash
tensorboard --logdir=/tmp/ros2learn/MARACollision-v0/ppo2_mlp --port 8008
```

# 참고문헌

- [ros2learn github](https://github.com/AcutronicRobotics/ros2learn)
- [ROS2Learn: a reinforcement learning framework for ROS 2](https://arxiv.org/pdf/1903.06282.pdf)