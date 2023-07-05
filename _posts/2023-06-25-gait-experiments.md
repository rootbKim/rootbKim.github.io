---
layout: post
title: Gait Rehabilitation Robot - Experimental results
feature-img: "assets/img/portfolio/gait.jpg"
thumbnail: "assets/img/portfolio/gait.jpg"
tags: [Gait Rehabilitation Robot, Robotics]
excerpt: "역동역학 기반의 파워어시스트 보행재활로봇인 DDgo Pro의 역동역학 기반 훈련 알고리즘의 검증을 위해 실험적으로 검증하고 그 결과를 정리한다."
---

## 1. 역동역학 해석의 검증

역동역학의 검증을 위해 Passive Mode 훈련 알고리즘에서 구현된 역동역학 기반의 PD 제어 수동 모드 알고리즘의 보행 훈련과, 역동역학을 기반으로 하지 않는 PD 제어 수동 모드 훈련을 비교한다.

실험은 70kg인 사람이 2km/h 속도의 보행을 대상으로 하였으며, 크랭크 각도에 따른 각도 추종 오차와 토크의 RMS 수치를 비교한다.

||Low gain PD control w/ I.D.|Low gain PD control w/o I.D.|High gain PD control w/o I.D.|
-|-|-|-|
Position RMS Error [deg]|6.9438|35.1362|14.3219|
RMS Torque [Nm]|22.1710|23.3602|27.1235|

위 결과는 역동역학을 기반으로 한 제어에서 낮은 게인으로도 설계된 참조 프로파일을 정확히 추종하면서 구동에 필요한 토크도 가장 작음을 알 수 있다. 이는 제안된 방법이 낮은 게인으로 훈련을 구현함으로써 보행 힘에 대해 유연성을 갖게 함과 동시에 참조 프로파일을 정확히 추종하여 보행 궤적을 익힐 수 있음을 검증할 수 있었다.

<img src="/assets/img/posts/230625_crank_torque_trajectory.png">

## 2. 근력 보조 성능 검증

### 2.1 Active Assisted Mode

Active Assisted Mode에서 구현한 의도에 따른 보행 보조 효과를 검증하고자 한다. 이를 위해 EMG 센서를 이용하여 근육 활성도를 보조 강도 별로 의도 토크 보조 유무에 따라 비교하였다.

실험은 2km/h를 목표 보행 속도로 설정하였으며, 피검자는 1분간 2km/h의 속도로 걷기 위한 의도를 갖고 보행 훈련을 진행하였다. 근육 활성도는 Tibialis anterior, Rectus femoris, Gastrocnemius, Biceps femoris의 근육 활성도를 측정하였다.

<img src="/assets/img/posts/230625_emg_sensor.png">

총 6회 측정하였으며, 보조 강도 $$k_s$$는 80%, 50%, 20%일 때의 의도 토크 $$T_{int}$$의 유무에 따른 근육 활성도를 비교하였다.

|Training Assistance Strength $$k_s$$|80%|50%|20%|
|-|-|-|-|
|Averaged signal[mV] w/o $$T_{int}$$|6.309|7.536|9.858|
|Averaged signal[mV] w/ $$T_{int}$$|5.875|7.114|8.245|
|Reduction rate|-6.88%|-5.60%|-16.36%|

<img src="/assets/img/posts/230625_aam_emg.png">

위 결과는 사용자의 의도에 따라 생성되는 의도 토크 $$T_{int}$$가 있을 때 사용자의 근육 활성도가 더 낮음을 알 수 있고, 또한 보조강도가 낮을 수록 높은 강도의 훈련을 할 수 있음을 보여준다. 이는 제안된 방법이 사용자의 의도를 적절히 관측하여 보행을 적절히 보조해 줌을 알 수 있고, 같은 목표 속도에 대해서 보조 강도가 낮을 수록 근육 활성도가 높아짐을 알 수 있다.

### 2.2 Active Mode

Active Mode 훈련에서 사용자가 보행할 때 필요한 근력 정도를 측정하였다. 2km/h의 보행 속도로 진행하였으며, Active Assisted Mode와 동일한 방법으로 측정하였다.

||Active mode training|Walking on the ground|
|-|-|-|
|Averaged signal[mV]|10.523|9.1059|

<img src="/assets/img/posts/230625_am_emg.png">

위 결과는 Active Mode 보행 훈련이 지면보행보다 근육 활성도가 약 15% 높음을 알 수 있다. 이러한 수치는 로봇의 운동을 보상했지만, 그 이외의 보상하지 못한 마찰로 인해 지면보행보다 높은 단계의 훈련을 하게 됨을 알 수 있다.

### 2.3 세 가지 모드의 근력 보조 성능 비교

세 가지 모드의 보행 훈련에 적용되는 보조 알고리즘이 적절하게 근육을 보조하는지 검증하기 위하여 각 모드에서의 EMG 테스트 결과를 비교하였다.

||PM|AAM 50%|AAM 20%|AM|
|-|-|-|-|-|
|Averaged signal[mV]|5.799|7.114|8.245|10.523|

<img src="/assets/img/posts/230625_3mode_emg.png">

실험 결과 근육 활성도가 Passive Mode에서 가장 낮으며, 훈련 강도가 높아질 수록 점점 높아짐을 알 수 있다. 이 검증을 통해 훈련 초기 단계에서부터 지면에서의 보행 단계 또는 그 이상의 훈련을 점진적으로 조절하여, 효과적인 훈련을 제공할 수 있음을 보였다.

## 3. 걸음새 개선 성능 검증

Active Assisted Mode의 모터 보조가 보행 자세의 개선 효과 여부를 측정하기 위하여 사용자의 환측 다리와 건측 다리의 보행 모션 유사성을 측정하였다.

> 건강한 성인의 경우 양측 다리의 보행 패턴이 대칭이지만 한 쪽 다리가 불편한 경우 환측 다리와 건측 다리의 보행 패턴이 비대칭이 된다.

이를 비교하기 위하여 Active Assisted Mode와 환자의 의도와 관계없이 지면에서의 보행 수준으로 보조해주는 Active Mode에서 환자의 의도에 따라 생성되는 모터 보조가 적절히 구현되었음을 검증하였다.

보행 유사성 실험을 위해 정상인을 대상으로 실험을 실시하였으며, 한쪽 다리에 무릎의 움직임을 제한하는 무릎고정장치를 부착했다.

실험은 Active Assisted Mode 2km/h, 50%일 때, 2km/h의 보행 속도로 훈련한 것과, Active Mode에서 2km/h의 보행 속도로 훈련한 것을 비교하였다. 비교 데이터는 환측 다리와 건측 다리의 스윙 단계에서의 크랭크 축의 각도에 따른 각속도 위상 궤적을 비교하였다. 양 다리의 위상 궤적의 데이터 유사성 정도를 비교하기 위해 상호 상관 관계 함수를 이용하여, 보행 유사성을 측정하였다. 상관 관계 값이 1에 가까울 수록 두 개의 데이터는 강한 양의 상관관계를 나타낸다.

||Active Assisted Mode|Active Mode|
|-|-|-|
|Cross correlation values|0.949|0.923|

* Active Assisted Mode에서의 양 다리의 위상 궤적

<img src="/assets/img/posts/230625_aam_gait_performance.png">

* Active Mode에서의 양 다리의 위상 궤적

<img src="/assets/img/posts/230625_am_gait_performance.png">

위 결과에서 Active Assisted Mode의 상호 상관 계수가 Active Mode의 상호 상관 계수보다 1에 가까운 것을 알 수 있고, Active Assisted Mode가 사용자의 의도에 따라 적절한 모터 보조가 이루어짐에 따라 걸음새 개선에 효과가 있음을 보여준다.
