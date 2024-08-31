---
layout: archive
title: Point in polygon - 다각형 내부 판별 알고리즘
tags: [Algorithm]
category: "Programming"
---

3개 이상의 point(x,y)로 구성된 다각형을 정의하는 방법과, 만들어진 다각형 내에 point가 존재하는지 판단하는 알고리즘에 대해 다룬다.

# 다각형 정의 방법

사용자로부터 다각형을 구성하기 위하여, (x,y) 좌표로 구성된 point 정보를 받아올 수 있다. 이 때, point의 순서가 순서대로 입력이 될 수도 있지만, 만약 순서를 제한하지 않는 경우 순서대로 입력되지 않을 수도 있다. 이런 경우에 point를 시계방향 또는 반시계방향으로 정렬을 시켜줘야 point 정보로 원하는 다각형의 형상을 만들 수 있다.

2차원 좌표로 구성되어있기 때문에, 단순히 오름차순이나 내림차순으로 정렬할 수 없고, 시계방향 또는 반시계방향으로 2차원 평면 상에서 정렬시켜야 한다.
주어진 모든 점에 대해서 정렬하기 위해서, 모든 점의 평균이 되는 점을 구한다. 그리고, 평균이 되는 점과 각각의 점과 각도를 계산하여, 이 각도가 작은 점부터 정렬을 하면 반시계방향 정렬, 각도가 큰 점부터 정렬을 하면 시계방향 정렬이 된다.

물론, 이 방법은 복잡한 다각형 형상에서는 완벽한 방법은 아닐 수 있지만, 단순한 다각형 형상에서는 잘 동작한다. 복잡한 다각형의 형상이나, 매우 많은 점에 대해서 외각선을 만들어주는 [컨벡스 헐](https://en.wikipedia.org/wiki/Convex_hull)과 같은 알고리즘을 사용할 수도 있다.

다음은 예제 코드로 C++ 기준으로 작성되었다.

```cpp
typedef struct
{
  float x;
  float y;
} Point;

std::vector<Point> sort(std::vector<Point> points)
{
    std::vector<Point> sort_points = points;
    Point avg{0, 0};

    for (auto it : sort_points) {
        avg.x += it.x;
        avg.y += it.y;
    }

    // 전체 점의 평균 점 계산
    avg.x = avg.x / sort_points.size();
    avg.y = avg.y / sort_points.size();

    // point sort
    std::sort(
        sort_points.begin(), sort_points.end(),
        [avg](Point lhs, Point rhs) -> bool
        {
        double lhs_ang = atan2(lhs.y - avg.y, lhs.x - avg.x);
        double rhs_ang = atan2(rhs.y - avg.y, rhs.x - avg.x);

        // 반시계방향
        return lhs_ang < rhs_ang;
        });

    return sort_points;
}
```

# Point in polygon

이제 다각형을 만들었으면, 임의의 점이 해당 다각형의 내부에 있는지 외부에 있는지 판단하고자 한다. 이를 판별하기 위해 [Point in polygon](https://en.wikipedia.org/wiki/Point_in_polygon) 알고리즘을 사용한다.

판별하고자 하는 점의 왼쪽 또는 오른쪽으로 반직선을 그었을 때 다각형과 마주치는 교점이 홀수개이면 다각형 내부에 존재하는 것이고, 짝수개이면 다각형 외부에 존재하는 것이다.

<img src="/assets/img/posts/221221_point_in_polygon.png">

Point in polygon 알고리즘을 이용하여 판단하는 C++ 코드는 다음과 같다.

```cpp
// 다각형 정보는 다음과 같이 주어졌다고 가정한다.
std::vector<Point> points = sort(input);

// is_inner_point 가 true면 내부, false면 외부
bool is_inner_point = false;

// point in polygon algorithm
// area points는 시계방향 또는 반시계 방향으로 정의되어 있어야 함.
for (std::size_t i = 0; i < points.size(); i++) {
    Point cur = points[i];
    Point prev = points[(i + 1) % points.size()];
    if ((cur.y < point.y && prev.y > point.y) || (prev.y < point.y && cur.y > point.y)) {
        if (point.x < (((prev.x - cur.x) / (prev.y - cur.y)) * (point.y - cur.y)) + cur.x) {
            is_inner_point = !is_inner_point;
        }
    }
}
```

# 참고문헌
* [시계방향, 반시계 방향 좌표 정렬](https://www.crocus.co.kr/1634)
* [컨벡스 헐 알고리즘(Convex Hull Algorithm)](https://www.crocus.co.kr/1288)
* [Point in polygon](https://en.wikipedia.org/wiki/Point_in_polygon)
* [다각형 내부 외부 판별 알고리즘](https://www.crocus.co.kr/1617)