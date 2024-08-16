---
layout: note_page
title: C++ Value Categories
tags: [C++]
category: "Programming"
---

C++에서 분류되는 값 카테고리(Value Category) 5가지에 대해서 정리하고, 표현식에 대한 값 카테고리를 검사하는 decltype 키워드에 대해서도 알아본다.

# 1. lvalue와 rvalue

모던 C++(C++11)로 넘어오기 이전 C++은 값을 좌측값(lvalue)와 우측값(rvalue)로 구분지어 표현하였다. 좌측값은 식의 좌변에 오는 값으로, 흔히 사용하는 변수의 개념으로서 이름을 가지고, 값을 할당하고, 변경할 수 있는 값을 말한다. 그에 반해 우측값은 식의 우변에 오는 값으로 어떠한 값도 저장하지 않고, 순수히 연산을 위해 사용되는 값이기 때문에 값을 할당하거나 변경할 수 있는 값이 아닌 값을 나타내었다.

# 2. Value Categories in Modern C++

모던 C++로 넘어오면서 이동의미론(move semantics)의 개념이 추가되면서 우측값 래퍼런스가 도입됨에 따라 다음 그림과 같이 값 카테고리를 5가지로 세분화되었다.

<img src="/assets/img/posts/230419_value_categories.png">

* `glvalue`(generalized lvalue): 메모리 상의 주소를 갖는 표현식으로 변수나 함수를 가리키는 표현식, 배열 요소, 포인터 등이 해당한다.
* `prvalue`(pure rvalue): 순수한 rvalue 값으로 구성된 표현식으로, 객체를 초기화하거나, 연산자의 피연산자의 값을 계산하는 표현식 등이다. 예를 들어, `5`나 `x+3` 등이 해당한다.
* `xvalue`(expiring value): 수명이 끝나가는 rvalue 객체를 가리키는 표현식으로, 주로 `move` 연산에 해당하는 표현식이다. 예를 들어, `std::move(x)`와 같은 표현식이 해당한다.
* `lvalue`: xvalue가 아닌 gvalue인 표현식에 해당한다.
* `rvalue`: xvalue 또는 prvalue인 표현식에 해당한다.

위의 5 가지 값 카테고리는 정체를 알 수 있는지, 이동 시킬 수 있는지에 따라 분류된다. 정체를 알 수 있다는 것은 다른 식과 구분할 수 있는 식이라는 것이다.

이러한 기준에 따라 분류하면 다음 표와 같다.

||이동 시킬 수 있음|이동 시킬 수 없음|
|-|-|-|
정체를 알 수 있음|xvalue|lvalue|
정체를 알 수 없음|prvalue|쓸모 없는 개념|

여기서 정체를 알 수 있는 모든 경우(xvalue, lvalue)는 glvalue로 분류되며, 이동 시킬 수 있는 모든 경우(xvalue, prvalue)는 rvalue로 분류된다.

위의 도식에서 볼 수 있듯이 xvalue, lvalue, prvalue는 `Primary Categories`이고, glvalue, rvalue는 `Mixed Categories`로 분류된다.

# 2.1 lvalue

이름을 가진 대부분의 객체는 lvalue이다. 이름을 가지기 때문에 정체를 알 수는 있지만 이동은 시킬 수 없는 값이다. lvalue는 대입 연산자의 왼쪽에 올 수 있는 값이며, &연산자로 주소를 가지고 올 수 있고, 대입 또는 복합 대입 연산자의 왼쪽 피연산자로 사용될 수 있다. 또한, lvalue를 lvalue로 초기화할 수 있으며, 이는 표현식으로 식별된 객체에 새 이름을 할당한다.

다음의 식은 lvalue 카테고리에 속하는 식들이다.

* 변수나 함수의 이름
* 데이터 멤버나 배열의 요소
* 좌측값 레퍼런스를 리턴하는 함수의 호출식: `std::cout << `, `++it`
* 복합 대입 연산자: `a = b`, `a += b`
* 전위 증감 연산자: `++a`, `--b`
* 문자열 리터럴: `"hi"`
* 멤버 변수의 참조

# 2.2 prvalue

prvalue는 이동시킬 수 있으나, 정체를 알 수 없는 값이다. 정체를 알 수 없기 때문에 lvalue와는 다르게 & 연산자로 주소값을 가질 수 없고, 식의 좌측에 올 수 없다. 단, 우측값 레퍼런스와 상수 좌측값 레퍼런스를 초기화하는데 사용할 수 있다. 예를 들어 다음과 같다.

```cpp
const int& pr = 42;
int&& prr = 42;
int& prrr = 42; // 에러
```

다음의 식은 prvalue 카테고리에 속하는 식들이다.

* 문자열 리터럴을 제외한 모든 리터럴: `42`, `true`, `nullptr`
* 레퍼런스가 아닌 것을 리턴하는 함수의 호출식: `str.substr(1, 2)`, `str1 + str2`
* 후위 증감 연산자: `a++`, `b--`
* 산술 연산자, 논리 연산자, 비교 연산자: `a + b`, `a % b`, `a & b`, `a << b`, `a && b`, `a || b`, `!a`, `a < b`, `a == b`, `a >= b`
* 주소값 연산자: `&a`
* `this`
* `enum`
* 람다 표현식(lambda expression) : `[](int x){ return x * x; }`

# 2.3 xvalue

xvalue는 lvalue처럼 정체를 알 수 있지만, rvalue처럼 이동도 시킬 수 있는 값 카테고리이다. 좌측값으로 분류되는 식을 이동시키기 위해 분류된 카테고리라고 이해하면 좋다. 표현식(expression)이 끝나고, 표현식이 의미하던 주소로 다시 접근했을 때, 값이 존재할 수도 있고 존재하지 않을 수도 있다(move가 가능하기 때문). 컴파일러가 prvalue의 임시 데이터를 저장할 공간이 필요한데, 이 때 사용하는 임시 데이터 객체가 xvalue이다.

xvalue는 `std::move(x)`와 같이 우측값 레퍼런스를 리턴하는 함수의 호출식과 같은 식이 포함된다. 따라서 이는 lvalue처럼 좌측값 레퍼런스를 초기화하는데 사용할 수도 있고, prvalue처럼 우측값 레퍼런스에 붙이거나 이동 생성자에 전달해서 이동시킬 수도 있다.

# 3. decltype

`decltype` 키워드를 이용하면 표현식 `(x)`에 대한 값 카테고리를 검사할 수 있다.

```cpp
decltype((x))
```

> 표현식 x가 실체를 가리키는 이름인 경우, 이름의 타입이 아닌 표현식 x의 타입을 얻고자 하기 때문에 decltype((x))에 괄호가 두 개이어야 한다. 예를 들어, 표현식 x가 단순히 변수 v를 나타낸다면, decltype(v)는 변수 v를 가리키는 표현식 x의 값 카테고리를 반영하는 타입이 아닌 변수 v의 타입을 반환한다.

decltyp 키워드에 전달된 표현식의 값 카테고리에 따라 다음과 같은 결과를 얻는다.

* (x)가 xvalue인 경우 decltype 는 `T&&` 가 된다.
* (x)가 lvalue인 경우 decltype 는 `T&` 가 된다.
* (x)가 prvalue인 경우 decltype 는 `T` 가 된다.

decltype을 사용하는 이유는 정확한 타입을 그대로 전달하기 위해 사용된다. `auto` 키워드를 이용하여 타입 추론을 할 수 있지만, auto는 정확한 타입을 표현해 주지는 않는다.

```cpp
const int i = 4;
auto j = i;         // int j = i;
decltype(i) k = i;  // const int k = i;
```

특히 템플릿 함수에서 어떤 객체의 타입이 템플릿 인자들에 의해서 결정되는 경우가 있다. 

```cpp
template <typename T, typename U>
void add(T t, U u, /* 무슨 타입이 와야 할까요? */ result) {
  *result = t + u;
}
```

위 함수에서 `result`의 타입이 `t + u` 의 결과에 의해 결정되는데, 이런 경우에 result 에 타입이 올 자리에 decltype 를 사용할 수 있다.


```cpp
template <typename T, typename U>
void add(T t, U u, decltype(t + u)* result) {
  *result = t + u;
}
```

만약 함수의 리턴값으로 더한 값을 바로 리턴한다면 다음과 같이 구현하면 된다. 

```cpp
template <typename T, typename U>
auto add(T t, U u) -> decltype(t + u) {
  return t + u;
}
```

# 참고문헌
* [cppreference - Value categories](https://en.cppreference.com/w/cpp/language/value_category)
* [Working Draft, Standard for Programming Language C++ - 7.2.1 Value category](http://eel.is/c++draft/basic.lval)
* [모두의 코드 씹어먹는 C++ - <16 - 3. 타입을 알려주는 키워드 decltype 와 친구 std::declval>](https://modoocode.com/294)
* [[C++] Value Categories (값 카테고리)](https://junstar92.tistory.com/471)
* [C++ Values(lvalue, rvalue, xvalue, prvalue, glvalue)](https://dydtjr1128.github.io/cpp/2019/06/10/Cpp-values.html)
* [[C++] Value category (pr-value, l-value, x-value, r-value, gl-value)](https://m42-orion.tistory.com/67)
* [C++ DevNote : 우측값과 좌측값 완벽 정리 (glvalue, rvalue, lvalue, xrvalue, prvalue)](https://koreanfoodie.me/1159)