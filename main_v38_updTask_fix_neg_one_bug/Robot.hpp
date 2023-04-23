#pragma once

#include "Obstacle.hpp"
#include "geo.hpp"
#include "utils.hpp"

using namespace std;

struct Robot
{
  const int id;
  int type = 0;  // 机器人类型，0表示运输型机器人，1表示攻击型机器人
  int state = 0; // 0 空闲 1 在买的路上 2在卖的路上
  int buy = -1;  // 机器人去哪个工作台买
  int buyPlatArea = 0;
  int sell = -1; // 机器人去哪个工作台卖
  int pid = -1;  // 当前所在工作台id
  int item = 0;  // 携带物品类型
  Point p;
  Point nextTo;
  Point preferredVelocity;
  D radius = R[0];
  D angle = 0;
  D lineSpeed = 0;
  Point v; // 速度
  D angleSpeed = 0;
  D tf = 0;                 // 时间系数
  D cf = 0;                 // 碰撞系数
  int vitualTargetPID = -1; // 虚拟工作台，当没有对手工作台，且机器人接不到任务时，则前往虚拟工作台
  bool attackJustRotate = false;

  int attackBuy123 = -1; //-1不需要购买123就去敌方工作台，>=0需要从对应的工作台购买但还没卖，-2已经买了
  int attackPlatId = -1;
  int attackFoeId = -1;
  std::vector<D> radar;
  std::vector<D> hisVelocity; // #
  int isStop = 0;             // 停止帧
  bool wait = false;
  int attackMode = 0; // 进攻型机器人的模式，0表示进攻模式，1表示防御模式

  std::vector<int> robotNeighbors, foeNeigbors;
  std::vector<int> path;
  std::string pathType;
  Robot(int id, D x, D y) : id(id), p(x, y), radar(360) {}

  void readState()
  {
    std::cin >> pid >> item >> tf >> cf;
    std::cin >> angleSpeed >> v.x >> v.y >> angle;
    std::cin >> p.x >> p.y;
    lineSpeed = v.len();
  }
  void readRadar()
  {
#ifndef SEMI_VERSION
    for (int i = 0; i < 360; ++i)
      std::cin >> radar[i];
#endif
  }
  void fresh() { radius = R[item > 0]; }
  int getTarget() const
  {
    if (state == 0)
    {
      if (vitualTargetPID == -1)
        return -2;
      return vitualTargetPID;
    }
    if (state == 1)
      return buy;
    if (sell < 0)
      return -2;
    return sell;
  }
  bool isAttacker() const { return type == 1; }
  bool isTransporter() const { return type == 0; }
  void updIsStop()
  {
    isStop = 0;
    // int K = 50;
    // double mnV = 0.08;
    // int sum = 0;
    // for (int i = max(0, int(hisVelocity.size()) - K);
    //      i < int(hisVelocity.size()); ++i)
    //   sum += hisVelocity[i] <= mnV;
    // if (sum == K)
    //   isStop = 15; // 刷新停止帧
  }
};