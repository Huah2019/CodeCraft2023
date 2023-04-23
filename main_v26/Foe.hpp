#pragma once
#include "geo.hpp"
#include "utils.hpp"
using namespace std;

struct Foe {
  Pt pos;      // 估计位置
  Pt velocity; // 估计速度，第一次发现速度直接估计为0
  D radius;    // 估计半径
  int visibleRobotMask;
  int ttl = 30; // 寿命
  Foe(Pt p, Pt v, D radius, int visibleRobotMask)
      : pos(p), velocity(v), radius(radius),
        visibleRobotMask(visibleRobotMask) {}
};
