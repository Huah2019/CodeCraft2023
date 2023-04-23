#pragma once
#include "geo.hpp"
#include "utils.hpp"

using std::abs;

struct Foe
{
  Pt pos;      // 估计位置
  Pt velocity; // 估计速度，第一次发现速度直接估计为0
  D radius;    // 估计半径
  int visibleRobotMask;
  Foe(Pt p, Pt v, D radius, int visibleRobotMask = 0) : pos(p), velocity(v), radius(radius), visibleRobotMask(visibleRobotMask) {}
};