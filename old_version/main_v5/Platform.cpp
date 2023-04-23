#include "Platform.hpp"
#include "Map.hpp"
#include "constants.hpp"

#define mp Map::instance()
void Platform::applyVisit(int rid) { applyVisitList.push_back(rid); }
void Platform::freshVisit()
{
  if (visitRobotId == -1)
    return;
  auto &rob = mp.robots[visitRobotId];
  if (rob.getTarget() != id && rob.p.dis(p) >= 1)
    visitRobotId = -1;
}
void Platform::processApplyVisitList()
{
  if (visitRobotId != -1)
    return;
  double mnScore = 1e9;
  for (int rid : applyVisitList)
  {
    auto &rob = mp.robots[rid];
    double delta = 1;
    double Score = delta;
    if (rob.item)
      Score += SELL_PRICE[rob.item] + delta;
    Score = rob.p.dis(p) / Score;
    if (Score < mnScore)
    {
      mnScore = Score;
      visitRobotId = rid;
    }
  }
  applyVisitList.clear();
}

bool Platform::canBuy(int item)
{
  if (!active)
    return false;
  // if (__builtin_popcount(willBuy) >= 1)
  //   return false;
  return (needBuy >> item & 1) && !((alreadyBuy | willBuy) >> item & 1);
}

bool Platform::isWaiting() { return prodTime == 0 && num == capacity; }

void Platform::readState()
{
  int _type;
  double _x;
  double _y;
  std::cin >> _type >> _x >> _y;
  assert(_type == type);
  assert(abs(p.x - _x) < 1e-6);
  assert(abs(p.y - _y) < 1e-6);
  std::cin >> prodTime >> alreadyBuy >> num;
}

bool Platform::buyItem(int item)
{
  assert(needBuy >> item & 1);
  if (!(alreadyBuy >> item & 1))
  {
    alreadyBuy |= 1 << item;
    if (willBuy >> item & 1)
      willBuy ^= 1 << item;
    return true;
  }
  return false;
}

int Platform::sellItem()
{
  if (!num)
    return 0;
  --num;
  --willSellNum;
  return sell;
}

int Platform::getNextTime()
{
  if (!active)
    return -1;
  if (willSellNum < num)
    return 0;
  if (willSellNum == num)
    return prodTime;
  return -1;
}
