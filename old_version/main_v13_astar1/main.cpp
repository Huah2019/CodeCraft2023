#include "Map.hpp"
#include "bits/stdc++.h"
#include "move.hpp"
#include "utils.hpp"
#include <algorithm>
#include <cctype>
#include <string>
using namespace std;

int sellNum[10];
double moneyJustLossTF = 200000;
double moneyJustLossCF = 200000;

void OK() { std::cout << "OK" << std::endl; }
void readOK()
{
  std::string s;
  std::cin >> s;
  myAssert(s == "OK");
}

auto &mp = Map::instance();
auto &mpFoe = Map::instanceFoe();

void initRobotType()
{
  if (mp.type == 2)
  {
    for (auto &r : mp.robots)
    {
      bool canExcuteTask = false;
      for (Platform &bplat : mp.plats)
      {
        int nextime = bplat.getNextTime();
        for (int area : bplat.validAreas)
        {
          D dis1 = std::max(mp.getEvalDis(r.p, bplat.id, area, 0.45),
                            1.0 * nextime / FPS * MAX_FORWARD_SPEED[mp.role]);
          if (dis1 >= 1e9 - 100)
            continue;

          for (Platform &splat : mp.plats)
          {
            if (splat.canBuy(bplat.sell))
            {
              D dis2 = mp.getEvalDisPP(bplat.id, area, splat.id);
              if (dis2 >= 1e9 - 100)
                continue;
              canExcuteTask = true;
              break;
            }
          }
        }
        if (canExcuteTask)
          break;
      }
      if (!canExcuteTask)
        r.type = 1;
    }
  }
  else if (mp.type == 3)
    mp.robots[3].type = 1;
  else if (mp.type == 4)
  {
    for (auto &r : mp.robots)
      r.type = 1;
  }
}

std::vector<std::vector<int>> getNewGrid(std::vector<std::string> g)
{
  int n = g.size();
  int gn = 2 * n + 1;
  vector<vector<int>> grid = vector<vector<int>>(gn, vector<int>(gn));
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j)
      if (g[i][j] == '#')
      {
        int gi = 2 * i + 1, gj = 2 * j + 1;
        for (int dx = -1; dx <= 1; ++dx)
          for (int dy = -1; dy <= 1; ++dy)
            grid[gi + dx][gj + dy] = -1;
      }
  for (int i = 0; i < gn; ++i)
    for (int j = 0; j < gn; ++j)
      if (i == 0 || i == gn - 1 || j == 0 || j == gn - 1)
        grid[i][j] = -1;
  auto newGrid = grid;
  for (int i = 1; i < gn - 1; ++i)
  {
    for (int j = 1; j < gn - 1; ++j)
      if (grid[i][j] != -1)
      {
        int k = j;
        while (k + 1 < gn - 1 && grid[i][k + 1] != -1)
          ++k;
        for (int h = j; h <= k; h += 2)
        {
          newGrid[i][h] = 1;
        }
        if (k - j == 4)
          newGrid[i][j + 2] = 0;
        j = k;
      }
  }
  for (int j = 1; j < gn - 1; ++j)
    for (int i = 1; i < gn - 1; ++i)
    {
      if (grid[i][j] != -1)
      {
        int k = i;
        while (k + 1 < gn - 1 && grid[k + 1][j] != -1)
          ++k;
        for (int h = i; h <= k; h += 2)
        {
          newGrid[h][j] = 1;
        }
        if (k - i == 4)
          newGrid[i + 2][j] = 0;
        i = k;
      }
    }
  return newGrid;
}

std::vector<std::vector<int>> newGrid;
void init()
{
  std::ios::sync_with_stdio(false);
  std::cin.tie(nullptr);
  std::cout << std::fixed << std::setprecision(12);
  std::cerr << std::fixed << std::setprecision(4);

  std::string str = "BLUE";
#ifndef SEMI_VERSION
  std::cin >> str;
#endif
  int role;
  if (str == "BLUE")
  {
    role = 0;
  }
  else
  {
    myAssert(str == "RED");
    role = 1;
  }
  std::vector<std::string> g(MAP_SIZE);
  for (int i = 0; i < MAP_SIZE; i++)
    std::cin >> g[i];

  auto &ng = newGrid = getNewGrid(g);

  std::vector<Pt> samplPts;
  for (int i = 0; i < ng.size(); i++)
  {
    for (int j = 0; j < ng.size(); j++)
    {
      if (ng[i][j] == 0)
      {
        samplPts.emplace_back(j * GSTEP, 50.0 - i * GSTEP);
      }
    }
  }

  mp.init(role, g, true, 3, samplPts);
  mpFoe.init(role ^ 1, g, false, 3, samplPts);

  initRobotType();
  Logger::instance().load("log_" + std::to_string(mp.mapId) + "_" + std::to_string(mp.role) + ".txt");

  readOK();
  OK();

  dbg("map type:", mp.type);

  dbg("init OK, time:", getTime());
}

int frameID, money;

void sellItem()
{
  for (Robot &r : mp.robots)
    if (r.state == 2 && r.pid == r.sell)
    {
      bool ok = mp.plats[r.pid].buyItem(r.item);
      if (ok)
      {
        ++sellNum[r.item];
        moneyJustLossTF += SELL_PRICE[r.item] * r.tf;
        moneyJustLossCF += SELL_PRICE[r.item] * r.cf;
        r.radius = 0.45;
        r.item = 0;
        r.state = 0;
        std::cout << "sell " << r.id << '\n';
      }
    }
}
void buyItem()
{
  for (Robot &r : mp.robots)
    if (r.state == 1 && r.pid == r.buy)
    {

      bool flag = true;

      if (r.buyPlatArea)
      {
        flag = checkPointArea(r.buyPlatArea, r.p - mp.plats[r.buy].p);
      }

      if (!flag)
        continue;

      double evalTime = frameID +
                        mp.plats[r.buy].p.dis(mp.plats[r.sell].p) /
                            MAX_FORWARD_SPEED[mp.role] * FPS +
                        15;
      if (evalTime > FRAME_LIMIT)
        continue;

      r.item = mp.plats[r.pid].sellItem();
      if (r.item != 0)
      {
        moneyJustLossTF -= BUY_PRICE[r.item];
        moneyJustLossCF -= BUY_PRICE[r.item];
        r.radius = 0.53;
        r.state = 2;
        std::cout << "buy " << r.id << '\n';
      }
    }
}

void updTaskForTransporter6()
{
  auto &robots = mp.robots;
  bool transmode = false;
  // if (mp.map_id == 4)
  //     transmode = true;

  std::vector<bool> ok(robots.size());
  int it = robots.size();
  if (frameID > FRAME_LIMIT - 200)
  {
    for (int i = 0; i < robots.size(); ++i)
      if (!robots[i].isTransporter() || robots[i].state)
        ok[i] = true;
  }
  else
  {
    for (auto &rob : robots)
      if (rob.isTransporter())
      {
        if (rob.state == 2)
        {
          if (transmode)
            mp.plats[rob.sell].willBuy ^= 1 << rob.item;
        }
        else if (rob.state == 1)
        {
          mp.plats[rob.sell].willBuy ^= 1 << mp.plats[rob.buy].sell;
          --mp.plats[rob.buy].willSellNum;
        }
      }
  }
  std::vector<int> num(10);
  for (auto &p : mp.plats)
    if (p.type == 7 && (p.alreadyBuy | p.willBuy))
    {
      for (int item = 4; item <= 6; ++item)
        if ((p.needBuy >> item & 1) &&
            !((p.alreadyBuy | p.willBuy) >> item & 1))
          --num[item];
      if (p.num == 0 && p.prodTime != -1 && p.prodTime <= 500)
        for (int item = 4; item <= 6; ++item)
          --num[item];
    }
    else if (p.type >= 4 && p.type <= 6)
    {
      int x = p.num + (p.prodTime != -1) - p.willSellNum;
      num[p.type] += x;
    }
  while (it--)
  {
    double mxScore = 0;
    int rid = -1, bpid = -1, spid = -1, rindex = -1, buyPlatArea = -1;
    for (size_t i = 0; i < robots.size(); ++i)
    {
      auto &rob = robots[i];
      if (!rob.isTransporter() || rob.state == 2 && !transmode)
        ok[i] = true;
      if (ok[i])
        continue;
      if (rob.state == 2)
      {
        for (auto &splat : mp.plats)
        {
          if (splat.canBuy(rob.item))
          {
            double dis = rob.p.dis(splat.p);
            double Score = SELL_PRICE[rob.item] / dis + 100000;

            int delta = 25;
            double evalTime =
                frameID + dis / MAX_FORWARD_SPEED[mp.role] * FPS + delta;

            if (Score > mxScore && evalTime <= FRAME_LIMIT)
            {
              mxScore = Score;
              rid = rob.id;
              rindex = i;
              bpid = -1;
              spid = splat.id;
            }
          }
        }
      }
      else
      {
        for (Platform &bplat : mp.plats)
        {
          int nextime = bplat.getNextTime();
          if (nextime == -1 || nextime > 50)
            continue;

          for (int area : bplat.validAreas)
          {
            D dis1 = std::max(mp.getEvalDis(rob.p, bplat.id, area, 0.45),
                              1.0 * nextime / FPS * MAX_FORWARD_SPEED[mp.role]);
            if (dis1 >= 1e9)
              continue;

            for (Platform &splat : mp.plats)
            {
              if (splat.canBuy(bplat.sell))
              {
                D dis2 = mp.getEvalDisPP(bplat.id, area, splat.id);
                if (dis2 >= 1e9)
                  continue;

                double Score =
                    (SELL_PRICE[bplat.sell] - BUY_PRICE[bplat.sell]) /
                    (dis1 + dis2);

                Score *= pow(
                    __builtin_popcount(splat.willBuy | splat.alreadyBuy) + 1,
                    1.5);

                if (bplat.type >= 1 && bplat.type <= 6 && splat.type == 9)
                  Score /= 10;

                if (bplat.type >= 4 && bplat.type <= 6 && splat.type == 9 &&
                    num[bplat.type] <= 1)
                  Score -= 10000;

                Score -= __builtin_popcount(bplat.willBuy) * 1000;

                if (num[splat.type] < 0)
                  Score += 10000;

                int delta = 25;
                double evalTime =
                    frameID + (dis1 + dis2) / MAX_FORWARD_SPEED[mp.role] * FPS +
                    delta;

                if (Score > mxScore && evalTime <= FRAME_LIMIT)
                {
                  mxScore = Score;
                  rid = rob.id;
                  rindex = i;
                  bpid = bplat.id;
                  spid = splat.id;
                  buyPlatArea = area;
                }
              }
            }
          }
        }
      }
    }
    if (rid == -1)
    {
      for (size_t i = 0; i < robots.size(); ++i)
        if (robots[i].isTransporter())
        {
          if (robots[i].state == 2)
            assert(ok[i]);
          else if (!ok[i])
          {
            robots[i].state = 0;
            robots[i].buy = robots[i].sell = -1;
          }
        }
      break;
    }
    ok[rindex] = true;
    auto &rob = robots[rid];
    if (rob.item)
    {
      assert(rob.state == 2);
      auto &splat = mp.plats[spid];
      rob.sell = spid;
      splat.willBuy |= 1 << rob.item;
    }
    else
    {
      auto &bplat = mp.plats[bpid];
      auto &splat = mp.plats[spid];
      rob.buy = bpid;
      rob.buyPlatArea = buyPlatArea;
      rob.sell = spid;
      ++bplat.willSellNum;
      splat.willBuy |= 1 << bplat.sell;
      if (splat.needBuy == (splat.willBuy | splat.alreadyBuy))
        ++num[splat.type];
      rob.state = 1;
    }
  }
}

void updTaskForTransporter7()
{
  auto &robots = mp.robots;
  bool transmode = false;
  // if (mp.map_id == 4)
  //     transmode = true;

  std::vector<bool> ok(robots.size());
  int it = robots.size();
  if (frameID > FRAME_LIMIT - 200)
  {
    for (int i = 0; i < robots.size(); ++i)
      if (!robots[i].isTransporter() || robots[i].state)
        ok[i] = true;
  }
  else
  {
    for (auto &rob : robots)
      if (rob.isTransporter())
      {
        if (rob.state == 2)
        {
          if (transmode)
            mp.plats[rob.sell].willBuy ^= 1 << rob.item;
        }
        else if (rob.state == 1)
        {
          mp.plats[rob.sell].willBuy ^= 1 << mp.plats[rob.buy].sell;
          --mp.plats[rob.buy].willSellNum;
        }
      }
  }
  std::vector<int> num(10);
  for (auto &p : mp.plats)
    if (p.type == 7) // todo: 检查7号工作台是否可以完成
    {
      for (int item = 4; item <= 6; ++item)
        if ((p.needBuy >> item & 1) &&
            !((p.alreadyBuy | p.willBuy) >> item & 1))
          --num[item];
      // if (p.num == 0 && p.prodTime != -1 && p.prodTime <= 500)
      //   for (int item = 4; item <= 6; ++item)
      //     --num[item];
    }
    else if (p.type >= 4 && p.type <= 6)
    {
      int x = p.num + (p.prodTime != -1) - p.willSellNum;
      num[p.type] += x;
    }
  // dbg(num[4], num[5], num[6]);

  while (it--)
  {
    double mxScore = -1e9;
    int rid = -1, bpid = -1, spid = -1, rindex = -1, buyPlatArea = -1;
    for (size_t i = 0; i < robots.size(); ++i)
    {
      auto &rob = robots[i];
      if (!rob.isTransporter() || rob.state == 2 && !transmode)
        ok[i] = true;
      if (ok[i])
        continue;
      if (rob.state == 2)
      {
        for (auto &splat : mp.plats)
        {
          if (splat.canBuy(rob.item))
          {
            double dis = rob.p.dis(splat.p);
            double Score = SELL_PRICE[rob.item] / dis + 100000;

            int delta = 25;
            double evalTime =
                frameID + dis / MAX_FORWARD_SPEED[mp.role] * FPS + delta;

            if (Score > mxScore && evalTime <= FRAME_LIMIT)
            {
              mxScore = Score;
              rid = rob.id;
              rindex = i;
              bpid = -1;
              spid = splat.id;
            }
          }
        }
      }
      else
      {
        for (Platform &bplat : mp.plats)
        {
          int nextime = bplat.getNextTime();
          if (nextime == -1 || nextime > 50)
            continue;

          for (int area : bplat.validAreas)
          {
            D dis1 = std::max(mp.getEvalDis(rob.p, bplat.id, area, 0.45),
                              1.0 * nextime / FPS * MAX_FORWARD_SPEED[mp.role]);
            if (dis1 >= 1e9)
              continue;

            for (Platform &splat : mp.plats)
            {
              if (splat.canBuy(bplat.sell))
              {
                D dis2 = mp.getEvalDisPP(bplat.id, area, splat.id);
                if (dis2 >= 1e9)
                  continue;

                double Score =
                    (SELL_PRICE[bplat.sell] - BUY_PRICE[bplat.sell]) /
                    (dis1 + dis2);

                Score *= pow(
                    __builtin_popcount(splat.willBuy | splat.alreadyBuy) + 1,
                    1.5);

                if (bplat.type >= 1 && bplat.type <= 6 && splat.type == 9)
                  Score /= 10;

                if (bplat.type >= 4 && bplat.type <= 6 && splat.type == 9 &&
                    num[bplat.type] <= 1)
                  Score -= 10000;

                Score -= __builtin_popcount(bplat.willBuy) * 1000;

                if (splat.type >= 4 && splat.type <= 6)
                  Score -= 10000 * num[splat.type];
                if (bplat.type >= 4 && bplat.type <= 6 && splat.type == 7)
                  Score -= 10000 * num[bplat.type];

                int delta = 25;
                double evalTime =
                    frameID + (dis1 + dis2) / MAX_FORWARD_SPEED[mp.role] * FPS +
                    delta;

                if (Score > mxScore && evalTime <= FRAME_LIMIT)
                {
                  mxScore = Score;
                  rid = rob.id;
                  rindex = i;
                  bpid = bplat.id;
                  spid = splat.id;
                  buyPlatArea = area;
                }
              }
            }
          }
        }
      }
    }
    if (rid == -1)
    {
      for (size_t i = 0; i < robots.size(); ++i)
        if (robots[i].isTransporter())
        {
          if (robots[i].state == 2)
            assert(ok[i]);
          else if (!ok[i])
          {
            robots[i].state = 0;
            robots[i].buy = robots[i].sell = -1;
          }
        }
      break;
    }
    ok[rindex] = true;
    auto &rob = robots[rid];
    if (rob.item)
    {
      assert(rob.state == 2);
      auto &splat = mp.plats[spid];
      rob.sell = spid;
      splat.willBuy |= 1 << rob.item;
    }
    else
    {
      auto &bplat = mp.plats[bpid];
      auto &splat = mp.plats[spid];
      rob.buy = bpid;
      rob.buyPlatArea = buyPlatArea;
      rob.sell = spid;
      ++bplat.willSellNum;
      splat.willBuy |= 1 << bplat.sell;
      if (splat.needBuy == (splat.willBuy | splat.alreadyBuy))
        ++num[splat.type];
      rob.state = 1;
    }
  }
}

void updTaskForAttacker()
{
  auto interfereFoe = [&](Robot &r)
  {
    r.attackFoeId = -1;
    if (mp.foes.empty())
      return;
    double mndis = 1e9;
    for (int id = 0; id < mp.foes.size(); ++id)
    {
      auto &e = mp.foes[id];
      if (!(e.visibleRobotMask >> r.id & 1))
        continue;
      // if (e.velocity.len() <= EPS)
      //   continue;
      double angle =
          normAngle(atan2(e.velocity.y, e.velocity.x) - e.pos.angleTo(r.p));
      // 参数，距离敌方机器人的距离
      if (e.pos.dis(r.p) < mndis)
      {
        mndis = e.pos.dis(r.p);
        if (std::abs(angle) <= PI / 2 || e.pos.dis(r.p) <= 0.5)
          mndis -= 3;
        r.attackFoeId = id;
      }
    }
  };

  auto &robots = mp.robots;
  for (auto &r : robots)
    if (r.isAttacker())
    {
      interfereFoe(r);
      if (r.attackFoeId != -1)
        continue;
      bool last = r.attackPlatId;
      if (frameID % 3000 == 0)
        r.attackPlatId = -1;
      if (r.attackPlatId == -1)
      {
        std::vector<int> pids;
        for (auto &p : mpFoe.plats)
          if (p.type == 7 && p.id != last)
            pids.push_back(p.id);
        if (pids.empty())
        {
          for (auto &p : mpFoe.plats)
            if (p.type == 9 && p.id != last)
              pids.push_back(p.id);
        }
        if (pids.empty())
        {
          for (auto &p : mpFoe.plats)
            if (p.type == 8 && p.id != last)
              pids.push_back(p.id);
        }
        if (pids.empty())
        {
          for (auto &p : mpFoe.plats)
            if (p.type >= 4 && p.type <= 6 && p.id != last)
              pids.push_back(p.id);
        }
        if (pids.empty())
        {
          for (auto &p : mpFoe.plats)
            if (p.id != last)
              pids.push_back(p.id);
        }
        if (pids.empty())
        {
          for (auto &p : mpFoe.plats)
            pids.push_back(p.id);
        }
        shuffle(pids.begin(), pids.end(), Random::instance().rng);
        double mndis = 1e9;
        for (int pid : pids)
        {
          double curdis = mpFoe.getEvalDis(r.p, pid, 0, r.radius);
          if (r.attackPlatId == -1 ||
              Random::randDouble(0.5, 1) > 1.0 * (curdis + 1) / (mndis + 1))
          {
            mndis = curdis;
            r.attackPlatId = pid;
          }
        }
      }
    }
}

void updTask()
{
  updTaskForTransporter6();
  // updTaskForTransporter7();
  updTaskForAttacker();
}

void processRadarInfo()
{
  auto &robots = mp.robots;
  std::vector<Foe> fs;

  for (size_t i = 0; i < robots.size(); ++i)
  {
    auto &r = robots[i];

    int robotNeighborMask = 0;

    auto isInMyRob = [&](Pt &p)
    {
      for (auto &o : robots)
      {
        if (!cmp(o.p.dis(p), o.radius))
        {
          robotNeighborMask |= 1 << o.id;
          return true;
        }
      }
      return false;
    };
    auto isObst = [&](int i, int j)
    {
      int x = MAP_SIZE - 1 - j, y = i;
      if (x < 0 || y < 0 || x >= MAP_SIZE || y >= MAP_SIZE ||
          mp.g[x][y] == '#')
      {
        return true;
      }
      return false;
    };
    auto isInObst = [&](const Pt &p)
    {
      int u = p.x * 2, v = p.y * 2;
      if (isObst(u, v))
        return true;
      if (!cmp(u, p.x * 2))
      {
        return isObst(u - 1, v);
      }
      if (!cmp(v, p.y * 2))
      {
        return isObst(u, v - 1);
      }
      return false;
    };
    for (int i = 0; i < 360; ++i)
    {
      int j = (i + 1) % 360, k = (i + 2) % 360;
      double angleI = r.angle + 1.0 * i / 180 * PI;
      double angleJ = r.angle + 1.0 * j / 180 * PI;
      double angleK = r.angle + 1.0 * k / 180 * PI;
      Point a = r.p + Point(cos(angleI), sin(angleI)) * r.radar[i];
      Point b = r.p + Point(cos(angleJ), sin(angleJ)) * r.radar[j];
      Point c = r.p + Point(cos(angleK), sin(angleK)) * r.radar[k];
      if (isInObst(a) || isInObst(b) || isInObst(c))
        continue;
      if (isInMyRob(a) || isInMyRob(b) || isInMyRob(c))
        continue;
      if (!sgn(a.cross(b, c)))
        continue;
      double radius = getRadius(a, b, c);
      double EPS1 = 0.00001; // 三点定圆精度设置1
      double EPS2 = 0.001;   // 三点定圆精度设置2
      if (std::abs(radius - 0.45) > EPS1 && std::abs(radius - 0.53) > EPS1)
        continue;
      // radius = std::abs(radius - 0.45) <= EPS1 ? 0.45 : 0.53;
      Pt center = getCenter(a, b, c);
      bool isNotIn = true;
      for (auto &rb : robots)
        if (center.dis(rb.p) <= EPS2)
        {
          isNotIn = false;
          break;
        }
      for (int j = 0; j < fs.size(); j++)
      {
        auto &e = fs[j];
        if (e.pos.dis(center) <= EPS2)
        {
          isNotIn = false;
          e.visibleRobotMask |= 1 << r.id;
        }
      }
      if (isNotIn)
      {
        fs.push_back(Foe(center, Pt(0, 0), radius, 1 << r.id));
      }
    }
    r.robotNeighbors.clear();
    for (int j = 0; j < robots.size(); j++)
    {
      if (robotNeighborMask >> j & 1)
      {
        r.robotNeighbors.push_back(j);
      }
    }
  }
  // 每帧最大移动距离为7/50=0.14
  for (auto &e : fs)
    for (auto &eOld : mp.foes)
      if (eOld.pos.dis(e.pos) <= 0.2)
        e.velocity = (e.pos - eOld.pos) * FPS;
  mp.foes = fs;

  for (auto &r : robots)
  {
    r.foeNeigbors.clear();
    for (int i = 0; i < fs.size(); i++)
    {
      auto &e = fs[i];
      if (e.pos.dis(r.p) <= FOE_NEIGHBOR_HORIZON)
      {
        r.foeNeigbors.push_back(i);
      }
    }
  }
  // if (int(enemys.size()) > 4)
  // {
  // if (mp.role == 1)
  // {
  //   dbg("enemys num:", mp.enemys.size());
  //   for (auto &e : mp.enemys)
  //     dbg(e.pos.str(), e.velocity.str(), e.radius);
  // }
  // dbg("my Robot:");
  // for (auto &r : robots)
  //   dbg(r.p.str(), r.v.str(), r.radius);
  // }
}
void work()
{
  while (std::cin >> frameID >> money)
  {
    int k;
    std::cin >> k;
    for (Platform &plat : mp.plats)
      plat.readState();
    for (Robot &rob : mp.robots)
      rob.readState();
    for (Robot &rob : mp.robots)
      rob.readRadar();

    readOK();

    // static D s1, s2, s3, s4, s5;
    std::cout << frameID << '\n';
    // D t1 = getTime();
    processRadarInfo();
    // D t2 = getTime();
    sellItem();
    // D t3 = getTime();
    updTask();
    // D t4 = getTime();
    buyItem();
    // D t5 = getTime();
    // if (frameID & 1 ^ 1)
    moveRobots();
    // D t6 = getTime();

    // s1 += t2 - t1;
    // s2 += t3 - t2;
    // s3 += t4 - t3;
    // s4 += t5 - t4;
    // s5 += t6 - t5;
    // static D sTime = 0;
    // auto tmp = [&] {
    //   D ss = getTime();
    //   int s = 0;
    //   for (auto e : mpFoe.graph[0]->edges) {
    //     bool flag = true;
    //     Pt a = mpFoe.nodes[e.u].p;
    //     Pt b = mpFoe.nodes[e.v].p;
    //     for (auto &f : mp.foes) {
    //       if (distPS(f.pos, a, b) <= f.radius + R[0]) {
    //         flag = false;
    //         break;
    //       }
    //     }
    //     if (flag) s++;
    //   }
    //   sTime += getTime() - ss;
    //   return s;
    // };

    // tmp();

    // if (frameID >= 9000 && mp.role) {
    //   auto &r = mp.robots[2];
    //   if (r.v.len() < 0.1) {

    //   if (std::abs(r.p.x - 42.3) < 0.1 || std::abs(r.p.x - 42.8) < 0.1) {
    //     std::cerr << "----------" << frameID << "---------\n";
    //     std::cerr << r.p.x << "\n";
    //     for (int i : r.path) {
    //       std::cerr << mp.nodes[i].p << "\n";
    //     }
    //   }
    //   }
    // }

    if (frameID >= 3400 && mp.role)
    {
      auto &r = mp.robots[2];
      Logger::instance().write("--------", frameID, r.pathType, "---------");
      Logger::instance().write(r.p);
      for (int i : r.path)
      {
        Logger::instance().write(mp.nodes[i].p);
      }

      if (frameID >= 3600)
        exit(0);
    }
    OK();
  }
}

int main()
{
  init();
  work();

  auto &logger = Logger::instance();
  std::string info = "Sell num for each item:";
  for (int i = 1; i <= 7; ++i)
    info += " " + std::to_string(sellNum[i]);
  logger.write(info);
  info = "Profit upper bound for each item:";
  for (int i = 1; i <= 7; ++i)
    info += " " + std::to_string(sellNum[i] * (SELL_PRICE[i] - BUY_PRICE[i]));
  logger.write(info);

  logger.write("money: " + std::to_string(money));
  int moneyUpperBound = 200000;
  for (int i = 1; i <= 7; ++i)
    moneyUpperBound += (SELL_PRICE[i] - BUY_PRICE[i]) * sellNum[i];
  logger.write("money upper bound: " + std::to_string(moneyUpperBound));
  logger.write("loss money: " + std::to_string(moneyUpperBound - money));
  logger.write("profit rate: " + std::to_string(1.0 * money / moneyUpperBound));
  logger.write("loss rate: " +
               std::to_string(1 - 1.0 * money / moneyUpperBound));
  logger.write("profit_just_loss_tf: " + std::to_string(moneyJustLossTF));
  logger.write("profit_just_loss_cf: " + std::to_string(moneyJustLossCF));
  return 0;
}