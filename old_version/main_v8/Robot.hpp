#pragma once

#include "geo.hpp"
#include "utils.hpp"
#include "Obstacle.hpp"


struct Robot {
  const int id;
  int state = 0; // 0 空闲 1 在买的路上 2在卖的路上
  int buy = -1;   // 机器人去哪个工作台买
  int buyPlatArea = 0;
  int sell = -1; // 机器人去哪个工作台卖
  int pid = -1;  // 当前所在工作台id
  int item = 0; // 携带物品类型
  Point p;
  Point nextTo;
  Point preferredVelocity;
  D radius = R[0];
  D angle = 0;
  D lineSpeed = 0;
  Point v; // 速度
  D angleSpeed = 0;
  D tf = 0; // 时间系数
  D cf = 0; // 碰撞系数
  std::vector<D> radar;
  bool wait = false;

  Robot(int id, D x, D y) : id(id), p(x, y), radar(360) {
  }

  void readState();
  void readRadar();
  void fresh();
  int getTarget();
};