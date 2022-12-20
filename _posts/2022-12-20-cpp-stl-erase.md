---
layout: post
title: C++ STL 컨테이너의 루프 내 삭제 방법
feature-img: "assets/img/posts/c++.png"
thumbnail: "assets/img/posts/c++.png"
tags: [C++]
---

C++에서 자주 사용되는 STL 컨테이너의 루프 내에서 요소를 삭제하는 방법에 대해서 다룬다. STL 컨테이너에 대하여 루프를 실행하다가, 특정 조건이 만족하는 경우 해당 iteration의 요소를 삭제하는 경우, 삭제만 수행하면 참조하고 있던 iteration이 삭제되므로, 루프 내 삭제 시 이러한 점을 유의해야 한다.

## 시퀀스 컨테이너(vector, deque, list)

다음 예제에서 `vector` 대신 다른 시퀀스 컨테이너를 사용하면 된다.

```cpp
vector<int> v;
vector<int>::iterator it = v.begin();
while(it != v.end()){
    if( ... ) {
        it = v.erase(it); // 특정 조건 만족 시 삭제
    } else {
        ++it;
    }
}
```

## 연관 컨테이너(map, multimap, set, unordered_map, unordered_set)

다음 예제에서 `map` 대신 다른 연관 컨테이너를 사용하면 된다.

```cpp
map<int, int> v;
map<int, int>::iterator it = v.begin();
while (it != v.end()) {
    if ( ... ) {
        v.erase(it++); // 특정 조건 만족 시 삭제
    } else {
        ++it;
    }
}
```

## 시퀀스 컨테이너와 연관 컨테이너의 차이

시퀀스 컨테이너는 `erase`의 반환 값이 다음 반복자를 반환하고, 연관 컨테이너는 `erase`시 반환 값이 없으므로 삭제 시 현재 반복자는 무효한 값이 된다. 따라서 `erase` 내에서 반복자를 증가시켜주어야 한다.


## 참고문헌
* [C++ - 각종 STL 컨테이너에서 루프 중 요소 삭제하기](https://jacking75.github.io/cpp_stl_container_remove/)