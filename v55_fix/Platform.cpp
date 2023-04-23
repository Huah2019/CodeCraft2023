#include "Platform.hpp"
#include "Map.hpp"
#include "constants.hpp"
using namespace std;

#define mp Map::instance()
void Platform::applyVisit(int rid)
{
  applyVisitList.push_back(rid);
}
void Platform::freshVisit()
{
  if (visitRobotId == -1)
    return;
  auto &rob = mp.robots[visitRobotId];
  if (rob.getTarget() != id && rob.p.dis(p) >= 1 || (--visitTLE) <= 0)
  {
    visitRobotId = -1, visitTLE = 0;
  }
  else
  {
    if (++tryApply >= 500)
    {
      tryApply = 0;
      disable = disableCons;
      disableCons = min(FRAME_LIMIT, disableCons * 2);
    }
  }
}
void Platform::processApplyVisitList()
{
  if (visitRobotId != -1)
    return;
  double mnScore = 1e9;
  if (!applyVisitList.empty())
  {
    if (++tryApply >= 500)
    {
      tryApply = 0;
      disable = disableCons;
      disableCons = min(FRAME_LIMIT, disableCons * 2);
    }
  }
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
      visitTLE = 100;
    }
  }
  applyVisitList.clear();
}

bool Platform::canBuy(int item)
{
  if (!active || disable)
    return false;
  if (unSafe)
  {
    if (rushAble == 0 || rushAble == 1 && type == 7)
      return false;
  }
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
  assert(std::abs(p.x - _x) < 1e-6);
  assert(std::abs(p.y - _y) < 1e-6);
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

int Platform::getNextTime(int robUnSafe, Pt rp)
{
  if (!active || disable)
    return -1;
  if (unSafe)
  {
    if (rushAble == 0 || rushAble == 1 && type >= 4 && type <= 7)
      return -1;
  }
  if (robUnSafe && rushAble <= 1 && type >= 4 && type <= 7 && rp.dis(p) <= 1)
    return -1;
  if (willSellNum < num)
    return 0;
  if (willSellNum == num)
    return prodTime;
  return -1;
}

void Platform::updSafety()
{

  if (unSafe > 0)
    --unSafe;
  int K = 50;              // 参数1，检测帧数
  double minDis = 0.5;     // 参数2，判定距离
  int dangerousFrame = 25; // 参数3，危险帧判定帧数
  int num = 0;
  for (int i = max(0, int(hisDist.size()) - K); i < int(hisDist.size()); ++i)
    if (hisDist[i] >= 0 && hisDist[i] <= minDis)
      ++num;
  if (num >= dangerousFrame)
  {
    if (unSafe == 0)
    {
      unSafe = unSafeFrame;
      unSafeFrame = min(FRAME_LIMIT, unSafeFrame * 2);
    }
    else
    {
      unSafe = unSafeFrame;
      unSafeFrame = min(FRAME_LIMIT, unSafeFrame + 2);
    }
  }
}