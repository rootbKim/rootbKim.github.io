---
layout: note_page
title: C++ 난수 생성 함수
tags: [C++]
category: "Programming"
---

C++에서 사용할 수 있는 난수 생성 함수에 대해서 다룬다.

## rand 함수

함수 원형

```cpp
int rand(void)
```

rand() 함수가 반환하는 값은 0~32767 사이의 값을 랜덤하게 반환하는 것이지만, rand() 함수는 프로그램이 생성될때 딱 값이 정해지기 때문에 프로그램을 여러번 실행시켜도 동일한 값이 나오게 된다.

## srand 함수

함수 원형

```cpp
void srand (unsigned int seed)
```

seed 값을 이용하여 rand 함수에 사용될 수를 초기화한다. 따라서 seed 값에 의해 rand 함수의 결과값이 변하게 된다.

하지만 srand의 seed값이 일정한것이 들어갔을때 일정한 rand 값이 나오게 되는 문제가 발생한다.

따라서 항상 변하는 seed 값을 이용해서 srand를 해줄 필요가 있고, 이를 위해 seed 값으로 시간 값을 이용한다.

## time 함수

함수 원형

```cpp
time_t time (time_t* timer);
```

UCT 기준 1970년 1월 1일 0시 0분 0초 부터 경과된 시간을 초(sec)로 반환하는 함수다.

time 함수의 반환 값을 seed로 사용하면 시간에 따라 rand의 값을 다르게 출력할 수 있다.

## C 스타일의 난수 생성 문제점

다음은 0~99 까지의 난수를 생성하는 코드

```cpp
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int main() {
  srand(time(NULL));

  for (int i = 0; i < 5; i++) {
    printf("난수 : %d \n", rand() % 100);
  }
  return 0;
}
```

하지만 위 코드 역시 마치 난수 처럼 보이는 의사 난수 (pseudo random number) 을 생성하는 코드이다. 컴퓨터 상에서 완전한 무작위로 난수를 생성하는 것은 생각보다 어렵습니다. 그 대신에, 첫 번째 수 만 무작위로 정하고, 나머지 수들은 그 수를 기반으로 여러가지 수학적 방법을 통해서 마치 난수 처럼 보이지만 실제로는 무작위로 생성된 것이 아닌 수열들을 만들기 때문이다.

위 코드의 문제점은 시드값이 초 단위로 너무 천천히 변한다는 것과, 균등하게 난수를 생성하지 않는다는 것이다. 또한 rand() 함수가 선형 합동 생성기 (Linear congruential generator) 이라는 알고리즘을 바탕으로 구현되어 있는데 이 알고리즘은 썩 좋은 품질의 난수열을 생성하지 못한다는 문제점이 있다.

## 균등한 난수 생성

```cpp
#include <iostream>
#include <random>

int main() {
  // 시드값을 얻기 위한 random_device 생성.
  std::random_device rd;

  // random_device 를 통해 난수 생성 엔진을 초기화 한다.
  std::mt19937 gen(rd());

  // 0 부터 99 까지 균등하게 나타나는 난수열을 생성하기 위해 균등 분포 정의.
  std::uniform_int_distribution<int> dis(0, 99);

  for (int i = 0; i < 5; i++) {
    std::cout << "난수 : " << dis(gen) << std::endl;
  }
}
```

C++ 에서는 좀더 양질의 시드값을 얻기 위해 random_device 라는 것을 제공한다. random_device 를 이용하면 운영체제 단에서 제공하는 진짜 난수를 사용할 수 있으나, 의사 난수보다 난수를 생성하는 속도가 매우 느리다. 따라서 시드값처럼 난수 엔진을 초기화 하는데 사용하고, 그 이후의 난수열은 난수 엔진으로 생성하는 것이 적합하다.

> 대부분의 운영체제에는 진짜 난수값들을 얻어낼 수 있는 여러가지 방식들을 제공하고 있습다. 예를 들어서 리눅스의 경우 /dev/random 나 /dev/urandom 을 통해서 난수값을 얻을 수 있다. 이 난수값은, 이전에 우리가 이야기 하였던 무슨 수학적 알고리즘을 통해 생성되는 가짜 난수가 아니라 정말로 컴퓨터가 실행하면서 마주치는 무작위적인 요소들 (예를 들어 장치 드라이버들의 noise) 을 기반으로한 진정한 난수를 제공한다.

std::mt19937 는 C++ random 라이브러리에서 제공하는 난수 생성 엔진 중 하나로, 메르센 트위스터 라는 알고리즘을 사용한다.

> 이 알고리즘은 기존에 rand 가 사용하였던 선형 합동 방식 보다 좀 더 양질의 난수열을 생성한다고 알려져있다. 무엇보다도 생성되는 난수들 간의 상관관계가 매우 작기 때문에 여러 시뮬레이션에서 사용할 수 있다.

`uniform_int_distribution<int>` 의 생성자를 이용하여 0 부터 99 까지 균등한 확률로 정수를 뽑아내기 위한 같이 균등 분포 (Uniform distribution) 객체를 정의한다..

마지막으로 균등 분포에 사용할 난수 엔진을 전달함으로써 균등 분포에서 무작위로 샘플을 뽑아낼 수 있다.

## 정규분포 난수 생성

normal_distribution 을 이용하여 평균 0 이고 표준 편차가 1 인 정규 분포를 정의하였다.

```cpp
#include <iomanip>
#include <iostream>
#include <map>
#include <random>

int main() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> dist(/* 평균 = */ 0, /* 표준 편차 = */ 1);

  for (int i = 0; i < 5; i++) {
    std::cout << "난수 : " << dist(gen) << std::endl;
  }
}
```

## 참고문헌

- [모두의 코드 씹어먹는 C++ - <17 - 3. 난수 생성(<random>)과 시간 관련 라이브러리(<chrono>) 소개>](https://modoocode.com/304)
- [개발자 지망생 [C언어/C++] rand, srand, time 랜덤함수에 대해서 (난수생성)](https://blockdmask.tistory.com/308)
