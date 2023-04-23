#pragma once

#include "geo.hpp"
#include "utils.hpp"
#include "constants.hpp"

using namespace std;

class Platform
{
public:
  const int id;
  const int type;
  const int needTime; // 生产需要的时间
  const int needBuy;
  const int sell;     // 销售物品的类型
  const int capacity; // 最多可以存多少销售物品
  const Point p;
  int prodTime = -1; // 生产剩余时间
  int alreadyBuy = 0;
  int willBuy = 0;
  int willSellNum = 0;
  int num = 0; // 当前存了多少销售物
  int visitRobotId = -1;
  bool rushAble = false; // #
  std::vector<int> applyVisitList;
  std::vector<double> hisDist; // #
  bool isSafe = true;          // #
  bool active = true;
  int lastUnsafe = -99999; // 工作台最后探测到的不安全时间
  int visitTLE = 0;        // 访问时限限制

  std::vector<int> validAreas;

  Platform(int id, int type, int needTime, int needBuy, int sell, int capacity,
           double x, double y)
      : id(id), type(type), needTime(needTime), needBuy(needBuy), sell(sell),
        capacity(capacity), p(x, y) {}
  void readState();
  bool buyItem(int);
  int sellItem();
  int getNextTime();
  bool canBuy(int);
  bool isWaiting();
  void applyVisit(int);
  void processApplyVisitList();
  void freshVisit();
  void updSafety(int);

  D getWaitDist()
  {
    return rushAble ? WAIT_DIST_LIMIT : 6;
  }
};