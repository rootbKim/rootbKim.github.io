---
layout: note_page
title: C++ 중첩 클래스
tags: [C++]
category: "Programming"
---

중첩 클래스에 대해서 알아보고, 중첩 클래스를 사용하는 이유와 예제에 대해 다룬다.

## 1. 중첩 클래스(Nested Class)

클래스 안에서 다른 클래스를 정의할 수 있는데, 특정 클래스 안에서만 사용하기 위한 용도로 사용되는 경우에 사용한다. 중첩 클래스를 사용함으로써 클래스를 그룹화할 수 있고, 클래스의 사용 범위를 제한할 수 있다.

```cpp
class OuterClass
{
  // ...
  class InnerClass;
  // ...
};
```

## 2. 중첩 클래스를 사용하는 이유

1. 클래스를 논리적으로 그룹화할 수 있다.
2. 특정 클래스 내부에서만 사용되기 때문에 코드를 더 쉽게 파악할 수 있으며, 유지 관리가 쉽다.
3. 특정 클래스 내부에서만 사용되므로 클래스 구조가 단순해지고, 사용자로부터 내부 클래스를 감출 수 있다.

## 3. 중첩 클래스 특징

- `OuterClass` 밖에서 `InnerClass`를 참조하기 위해서 `OuterClass::` 스코프 지정자를 사용해야 한다.
- 위의 `InnerClass`처럼 `OuterClass` 에서 `InnerClass`를 선언만 하고, 정의 코드는 따로 작성할 수 있다.
- `InnerClass`에서 `OuterClass`의 `private` 멤버에 접근이 가능하다.

## 4. 중첩 클래스 예제

다음 예제는 `open-rmf`에서 개발 중인 [rmf_traffic](https://github.com/open-rmf/rmf_traffic) 라이브러리의 geometry 부분의 일부를 가져왔다.

`Shape.hpp`는 geometry 패키지의 베이스 클래스로, Box나 Circle, Polygon 등의 클래스의 베이스 클래스가 된다. `Shape` 클래스는 `Internal`이란 이름의 중첩 클래스를 갖는데, `Internal` 클래스는 `ShapeInternal.hpp`에 정의되어 있다. 이 `Internal` 클래스는 [Flexible Collision Library(`fcl`)](https://github.com/flexible-collision-library/fcl)을 이용한 클래스로, 이 클래스 또한 상속되어 각각의 형태로 `fcl` 객체로 생성된다.

사용자가 fcl 라이브러리에 직접 접근하는 것을 막고, 개발자가 정의한 형태의 geometry만을 사용하도록 하기 위하여 중첩 클래스를 사용한 것으로 판단된다.

`Shape` 클래스는 `Internal` 클래스 객체를 unique 포인터로 갖고 있으며, 이에 대한 접근 제어 함수가 정의되어 있다.

```cpp
// Shape.hpp

class Shape
{
public:

  Shape(Shape&&) = delete;
  Shape& operator=(Shape&&) = delete;

  /// \internal
  class Internal;

  /// \internal
  Internal* _get_internal();

  /// \internal
  const Internal* _get_internal() const;

  virtual ~Shape();

protected:

  /// \internal
  Shape(std::unique_ptr<Internal> internal);

private:

  std::unique_ptr<Internal> _internal;

};
```

```cpp
// ShapeInternal.hpp

#include <Shape.hpp>
#include <fcl/collision_object.h>
#include <vector>

class Shape::Internal
{
public:
  using CollisionGeometryPtr = std::shared_ptr<fcl::CollisionGeometry>;
  using CollisionGeometries = std::vector<CollisionGeometryPtr>;

  virtual CollisionGeometries make_fcl() const = 0;
};
```

```cpp
#include <Shape.hpp>
#include <ShapeInternal.hpp>

Shape::Internal* Shape::_get_internal()
{
  return _internal.get();
}

const Shape::Internal* Shape::_get_internal() const
{
  return _internal.get();
}

Shape::Shape(std::unique_ptr<Internal> internal)
  : _internal(std::move(internal))
{
  // Do nothing
}

Shape::~Shape()
{
  // Do nothing
}
```


## 참고문헌

- [[C#]중첩 클래스(Nested Class)](https://developer-talk.tistory.com/473)
- [open-rmf/rmf_traffic github pages](https://github.com/open-rmf/rmf_traffic)