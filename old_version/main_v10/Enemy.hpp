#pragma once
#include "geo.hpp"
#include "utils.hpp"

struct Enemy
{
    Pt pos;               // 估计位置
    Pt velocity;          // 估计速度，第一次发现速度直接估计为0
    double radius = 0.53; // 估计半径
    int visitIt;          // 可以看到它的机器人集合
};