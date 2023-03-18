---
layout: post
title: gym-gazebo2 패키지 분석
feature-img: "assets/img/portfolio/alr_logo.png"
thumbnail: "assets/img/portfolio/alr_logo.png"
tags: [Gym Gazebo]
---

## 1. gg_random.py

해당 코드는 강화학습을 위한 코드는 아니고, gym-gazebo2 환경을 학습하기 위한 코드라고 한다. 정의된 `MARA-v0` 환경을 등록하고, 랜덤한 `action`을 부여하는 코드라고 보면 될 것 같다.

```python
import gym
import gym_gazebo2
import time
env = gym.make('MARA-v0')

while True:
    # take a random action
    observation, reward, done, info = env.step(env.action_space.sample())
```

## 2. Register

`~/gym_gazebo2/gym_gazebo2/__init__.py`에 다음과 같이 `MARAEnv`가 register되어 있다.

```python
from gym.envs.registration import register

register(
    id='MARA-v0',
    entry_point='gym_gazebo2.envs.MARA:MARAEnv',
)
```

## 3. MARAEnv

위에서 가져온 환경인 `MARA-v0`는 gym-gazebo2 패키지 내에 `MARAEnv`라는 클래스로 정의되어 있다.

import된 라이브러리들을 보면, `gym`과 관련된 라이브러리와, `gym_gazebo2`와 `gym`에 필요한 유틸리티 라이브러리, 그리고 ROS와 GAZEBO를 위한 `rclpy`, `gazebo_msgs`, 기타 인터페이스 패키지들을 가져온 것을 볼 수 있다.

그리고 `MARAEnv` 클래스는 `gym.Env` 클래스를 상속받는다.

```python
# gym_gazebo2/envs/MARA/mara.py

import gym
gym.logger.set_level(40) # hide warnings
import time
import numpy as np
import copy
import math
import os
import psutil
import signal
import sys
from scipy.stats import skew
from gym import utils, spaces
from gym_gazebo2.utils import ut_generic, ut_launch, ut_mara, ut_math, ut_gazebo, tree_urdf, general_utils
from gym.utils import seeding
from gazebo_msgs.srv import SpawnEntity
import subprocess
import argparse
import transforms3d as tf3d

# ROS 2
import rclpy


from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # Used for publishing mara joint angles.
from control_msgs.msg import JointTrajectoryControllerState
# from gazebo_msgs.srv import SetEntityState, DeleteEntity
from gazebo_msgs.msg import ContactState, ModelState#, GetModelList
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from ros2pkg.api import get_prefix_path
from builtin_interfaces.msg import Duration

# Algorithm specific
from PyKDL import ChainJntToJacSolver # For KDL Jacobians

class MARAEnv(gym.Env):
```

### 3.1 init

초기화 시 ros node를 만들고, target_position, target_orientation의 범위의 지정, 각 조인트와 링크 선언, end effectr의 위치와 속도, 초기 위치 등의 변수들을 선언한다. `_pub`은 GAZEBO 상에 각 조인트의 정보를 publish하고, `_sub`은 GAZEBO 상에서의 Joint 상태를 subscription 한다. `_sub_coll`은 충돌 결과를 sub하고, reset_sim은 reset 시 simulation 환경을 reset을 요청하는 service client이다. `action_space`와 `observation_space`를 선언하고, `spawn_cli`를 이용하여 초기 위치에 대하여 GAZEBO상에 spawn한다.

```python
    def __init__(self):
        """
        Initialize the MARA environemnt
        """
        # Manage command line args
        args = ut_generic.getArgsParserMARA().parse_args()
        self.gzclient = args.gzclient
        self.realSpeed = args.realSpeed
        # self.realSpeed = True
        self.velocity = args.velocity
        self.multiInstance = args.multiInstance
        self.port = args.port

        # Set the path of the corresponding URDF file
        if self.realSpeed:
            urdf = "reinforcement_learning/mara_robot_run.urdf"
            urdfPath = get_prefix_path("mara_description") + "/share/mara_description/urdf/" + urdf
        else:
            urdf = "reinforcement_learning/mara_robot_train.urdf"
            urdfPath = get_prefix_path("mara_description") + "/share/mara_description/urdf/" + urdf

        # Launch mara in a new Process
        self.launch_subp = ut_launch.startLaunchServiceProcess(
            ut_launch.generateLaunchDescriptionMara(
                self.gzclient, self.realSpeed, self.multiInstance, self.port, urdfPath))

        # Create the node after the new ROS_DOMAIN_ID is set in generate_launch_description()
        rclpy.init()
        self.node = rclpy.create_node(self.__class__.__name__)

        # class variables
        self._observation_msg = None
        self.max_episode_steps = 1024 #default value, can be updated from baselines
        self.iterator = 0
        self.reset_jnts = True
        self._collision_msg = None

        #############################
        #   Environment hyperparams
        #############################
        # Target, where should the agent reach
        self.targetPosition = np.asarray([-0.40028, 0.095615, 0.25]) # close to the table
        # self.targetPosition =  np.asarray([round(np.random.uniform(-0.615082, -0.35426), 5), round(np.random.uniform( -0.18471, 0.1475), 5), 0.35])
        self.target_orientation = np.asarray([0., 0.7071068, 0.7071068, 0.]) # arrow looking down [w, x, y, z]
        # self.targetPosition = np.asarray([-0.386752, -0.000756, 1.40557]) # easy point
        # self.target_orientation = np.asarray([-0.4958324, 0.5041332, 0.5041331, -0.4958324]) # arrow looking opposite to MARA [w, x, y, z]

        EE_POINTS = np.asmatrix([[0, 0, 0]])
        EE_VELOCITIES = np.asmatrix([[0, 0, 0]])

        # Initial joint position
        INITIAL_JOINTS = np.array([0., 0., 0., 0., 0., 0.])

        # # Topics for the robot publisher and subscriber.
        JOINT_PUBLISHER = '/mara_controller/command'
        JOINT_SUBSCRIBER = '/mara_controller/state'


        # joint names:
        MOTOR1_JOINT = 'motor1'
        MOTOR2_JOINT = 'motor2'
        MOTOR3_JOINT = 'motor3'
        MOTOR4_JOINT = 'motor4'
        MOTOR5_JOINT = 'motor5'
        MOTOR6_JOINT = 'motor6'
        EE_LINK = 'ee_link'

        # Set constants for links
        WORLD = 'world'
        BASE = 'base_robot'
        MARA_MOTOR1_LINK = 'motor1_link'
        MARA_MOTOR2_LINK = 'motor2_link'
        MARA_MOTOR3_LINK = 'motor3_link'
        MARA_MOTOR4_LINK = 'motor4_link'
        MARA_MOTOR5_LINK = 'motor5_link'
        MARA_MOTOR6_LINK = 'motor6_link'
        EE_LINK = 'ee_link'

        JOINT_ORDER = [MOTOR1_JOINT,MOTOR2_JOINT, MOTOR3_JOINT,
                        MOTOR4_JOINT, MOTOR5_JOINT, MOTOR6_JOINT]
        LINK_NAMES = [ WORLD, BASE,
                        MARA_MOTOR1_LINK, MARA_MOTOR2_LINK,
                        MARA_MOTOR3_LINK, MARA_MOTOR4_LINK,
                        MARA_MOTOR5_LINK, MARA_MOTOR6_LINK, EE_LINK]

        reset_condition = {
            'initial_positions': INITIAL_JOINTS,
             'initial_velocities': []
        }
        #############################

        m_jointOrder = copy.deepcopy(JOINT_ORDER)
        m_linkNames = copy.deepcopy(LINK_NAMES)

        # Initialize target end effector position
        self.environment = {
            'jointOrder': m_jointOrder,
            'linkNames': m_linkNames,
            'reset_conditions': reset_condition,
            'tree_path': urdfPath,
            'end_effector_points': EE_POINTS,
        }

        # Subscribe to the appropriate topics, taking into account the particular robot
        self._pub = self.node.create_publisher(JointTrajectory, JOINT_PUBLISHER, qos_profile=qos_profile_sensor_data)
        self._sub = self.node.create_subscription(JointTrajectoryControllerState, JOINT_SUBSCRIBER, self.observation_callback, qos_profile=qos_profile_sensor_data)
        self._sub_coll = self.node.create_subscription(ContactState, '/gazebo_contacts', self.collision_callback, qos_profile=qos_profile_sensor_data)
        self.reset_sim = self.node.create_client(Empty, '/reset_simulation')

        # Initialize a tree structure from the robot urdf.
        #   note that the xacro of the urdf is updated by hand.
        # The urdf must be compiled.
        _, self.ur_tree = tree_urdf.treeFromFile(self.environment['tree_path'])
        # Retrieve a chain structure between the base and the start of the end effector.
        self.mara_chain = self.ur_tree.getChain(self.environment['linkNames'][0], self.environment['linkNames'][-1])
        self.numJoints = self.mara_chain.getNrOfJoints()
        # Initialize a KDL Jacobian solver from the chain.
        self.jacSolver = ChainJntToJacSolver(self.mara_chain)

        self.obs_dim = self.numJoints + 6 + 12

        # # Here idially we should find the control range of the robot. Unfortunatelly in ROS/KDL there is nothing like this.
        # # I have tested this with the mujoco enviroment and the output is always same low[-1.,-1.], high[1.,1.]

        low = -np.pi * np.ones(self.numJoints)
        high = np.pi * np.ones(self.numJoints)

        self.action_space = spaces.Box(low, high)

        high = np.inf*np.ones(self.obs_dim)
        low = -high
        self.observation_space = spaces.Box(low, high)


        spawn_cli = self.node.create_client(SpawnEntity, '/spawn_entity')

        while not spawn_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('/spawn_entity service not available, waiting again...')

        modelXml = ut_gazebo.getTargetSdf()
        pose = Pose()
        pose.position.x = self.targetPosition[0]
        pose.position.y = self.targetPosition[1]
        pose.position.z = self.targetPosition[2]
        pose.orientation.x = self.target_orientation[1]
        pose.orientation.y= self.target_orientation[2]
        pose.orientation.z = self.target_orientation[3]
        pose.orientation.w = self.target_orientation[0]

        #override previous spawn_request element.
        self.spawn_request = SpawnEntity.Request()
        self.spawn_request.name = "target"
        self.spawn_request.xml = modelXml
        self.spawn_request.robot_namespace = ""
        self.spawn_request.initial_pose = pose
        self.spawn_request.reference_frame = "world"

        # #ROS2 Spawn Entity
        target_future = spawn_cli.call_async(self.spawn_request)
        rclpy.spin_until_future_complete(self.node, target_future)
        # Seed the environment
        self.seed()
        self.buffer_dist_rewards = []
        self.buffer_tot_rewards = []
        self.collided = 0
```

### 3.2 observation_callback, collision_callback, set_episode_size

각각 GAZEBO Joint 결과와, 충돌 여부를 콜백받고, episode 크기를 결정하는 메소드이다.

```python
    def observation_callback(self, message):
        """
        Callback method for the subscriber of JointTrajectoryControllerState
        """
        self._observation_msg = message

    def collision_callback(self, message):
        """
        Callback method for the subscriber of Collision data
        """
        collision_messages = ["mara::base_robot::base_robot_collision", "ground_plane::link::collision"]
        if message.collision1_name != message.collision2_name:
            if not ((message.collision1_name in collision_messages) and (message.collision2_name in collision_messages)):
                self._collision_msg = message

    def set_episode_size(self, episode_size):
        self.max_episode_steps = episode_size
```

### 3.3 take_observation

`take_observation()`은 JointTrajectoryControllerState 결과를 이용하여 F.K. 계산 및 Jacobian 계산을 통하여 EE의 위치, 속도를 계산한다.

```python
    def take_observation(self):
        """
        Take observation from the environment and return it.
        :return: state.
        """
        # # # # Take an observation
        rclpy.spin_once(self.node)
        obs_message = self._observation_msg

        # Check that the observation is not prior to the action
        # obs_message = self._observation_msg
        while obs_message is None or int(str(self._observation_msg.header.stamp.sec)+(str(self._observation_msg.header.stamp.nanosec))) < self.ros_clock:
            # print("I am in obs_message is none")
            rclpy.spin_once(self.node)
            obs_message = self._observation_msg

        # Collect the end effector points and velocities in cartesian coordinates for the processObservations state.
        # Collect the present joint angles and velocities from ROS for the state.
        lastObservations = ut_mara.processObservations(obs_message, self.environment)
        #Set observation to None after it has been read.
        self._observation_msg = None

        # Get Jacobians from present joint angles and KDL trees
        # The Jacobians consist of a 6x6 matrix getting its from from
        # (joint angles) x (len[x, y, z] + len[roll, pitch, yaw])
        ee_link_jacobians = ut_mara.getJacobians(lastObservations, self.numJoints, self.jacSolver)
        if self.environment['linkNames'][-1] is None:
            print("End link is empty!!")
            return None
        else:
            translation, rot = general_utils.forwardKinematics(self.mara_chain,
                                                self.environment['linkNames'],
                                                lastObservations[:self.numJoints],
                                                baseLink=self.environment['linkNames'][0], # use the base_robot coordinate system
                                                endLink=self.environment['linkNames'][-1])

            current_eePos_tgt = np.ndarray.flatten(general_utils.getEePoints(self.environment['end_effector_points'], translation, rot).T)
            eePos_points = current_eePos_tgt - self.targetPosition

            eeVelocities = ut_mara.getEePointsVelocities(ee_link_jacobians, self.environment['end_effector_points'], rot, lastObservations)

            # Concatenate the information that defines the robot state
            # vector, typically denoted asrobot_id 'x'.
            state = np.r_[np.reshape(lastObservations, -1),
                          np.reshape(eePos_points, -1),
                          np.reshape(eeVelocities, -1),
                          np.reshape(current_eePos_tgt,-1),
                          np.reshape(rot.reshape(1, 9),-1)]

            return state
```

### 3.4 collision

`collision()`은 Gazebo로부터 받은 충돌 메세지를 확인하여, 환경을 리셋하는 메소드이다.

```python
    def collision(self):
        # Reset if there is a collision
        if self._collision_msg is not None:
            while not self.reset_sim.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('/reset_simulation service not available, waiting again...')

            reset_future = self.reset_sim.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self.node, reset_future)
            self._collision_msg = None
            self.collided += 1
            return True
        else:
            return False
```

### 3.5 seed

`seed()` 메소드는 초기 위치와 목적지 위치를 랜덤하게 하기 위한 메소드이다.

```python
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
```

### 3.6 step

`step()` 메소드는 각 단계별로 액션을 수행하는 메소드이다. Gazebo에 action을 수행하도록 publish한다.

다음으로, Gazebo에서 수행된 결과를 가지고 그 `obs`를 계산하고, `obs`에 대한 reward를 계산 및 충돌 여부를 업데이트한다.

`done = bool(self.iterator == self.max_episode_steps)` 명령으로 `max_episode_steps`에 도달한 경우 `done`을 true로 설정한다. 이 경우에, info 정보를 담아서 리턴해준다.

그리고 각 iteration 마다 `obs, reward, done, info`를 리턴한다.

```python
    def step(self, action):
        """
        Implement the environment step abstraction. Execute action and returns:
            - action
            - observation
            - reward
            - done (status)
        """
        self.iterator+=1
        # Execute "action"
        self._pub.publish(ut_mara.getTrajectoryMessage(
            action[:self.numJoints],
            self.environment['jointOrder'],
            self.velocity))

        self.ros_clock = rclpy.clock.Clock().now().nanoseconds

        # Take an observation
        obs = self.take_observation()
        # Fetch the positions of the end-effector which are nr_dof:nr_dof+3
        rewardDist = ut_math.rmseFunc( obs[self.numJoints:(self.numJoints+3)] )

        collided = self.collision()

        reward = ut_math.computeReward(rewardDist)

        # Calculate if the env has been solved
        done = bool(self.iterator == self.max_episode_steps)

        self.buffer_dist_rewards.append(rewardDist)
        self.buffer_tot_rewards.append(reward)
        info = {}
        if self.iterator % self.max_episode_steps == 0:

            max_dist_tgt = max(self.buffer_dist_rewards)
            mean_dist_tgt = np.mean(self.buffer_dist_rewards)
            std_dist_tgt = np.std(self.buffer_dist_rewards)
            min_dist_tgt = min(self.buffer_dist_rewards)
            skew_dist_tgt = skew(self.buffer_dist_rewards)

            max_tot_rew = max(self.buffer_tot_rewards)
            mean_tot_rew = np.mean(self.buffer_tot_rewards)
            std_tot_rew = np.std(self.buffer_tot_rewards)
            min_tot_rew = min(self.buffer_tot_rewards)
            skew_tot_rew = skew(self.buffer_tot_rewards)

            num_coll = self.collided

            info = {"infos":{"ep_dist_max": max_dist_tgt,"ep_dist_mean": mean_dist_tgt,"ep_dist_min": min_dist_tgt,\
                "ep_rew_max": max_tot_rew,"ep_rew_mean": mean_tot_rew,"ep_rew_min": min_tot_rew,"num_coll": num_coll,\
                "ep_dist_skew": skew_dist_tgt,"ep_dist_std": std_dist_tgt, "ep_rew_std": std_tot_rew, "ep_rew_skew":skew_tot_rew}}
            self.buffer_dist_rewards = []
            self.buffer_tot_rewards = []
            self.collided = 0

        # Return the corresponding observations, rewards, etc.
        return obs, reward, done, info
```

### 3.7 reset

`reset()`이 호출되면 iterator를 0으로 초기화, GAZEBO를 초기화하고 초기화에 대한 observation을 리턴한다.

```python
    def reset(self):
        """
        Reset the agent for a particular experiment condition.
        """
        self.iterator = 0

        if self.reset_jnts is True:
            # reset simulation
            while not self.reset_sim.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('/reset_simulation service not available, waiting again...')

            reset_future = self.reset_sim.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self.node, reset_future)

        self.ros_clock = rclpy.clock.Clock().now().nanoseconds

        # Take an observation
        obs = self.take_observation()

        # Return the corresponding observation
        return obs
```

### 3.8 close

`close()`를 정의하며, ROS 노드를 종료시키고, 관련된 환경들을 종료시킨다.

```python
    def close(self):
        print("Closing " + self.__class__.__name__ + " environment.")
        self.node.destroy_node()
        parent = psutil.Process(self.launch_subp.pid)
        for child in parent.children(recursive=True):
            child.kill()
        rclpy.shutdown()
        parent.kill()
```

이상으로 `MARAEnv`에 대해서 간략하게 정리하였는데, utility 관련된 부분이나, F.K. 및 Jacobian 계산 등 더 깊은 이해가 필요할 것이다.

## 참고문헌

- [gym-gazebo2 깃헙 페이지](https://github.com/AcutronicRobotics/gym-gazebo2)
