#include "Map.hpp"
#include "bits/stdc++.h"
#include "move.hpp"
#include "utils.hpp"
#include <string>
using namespace std;

int sellNum[10];
double moneyJustLossTF = 200000;
double moneyJustLossCF = 200000;

auto &mp = Map::instance();
auto &mpFoe = Map::instanceFoe();

// 全局区 begin
// updTaskForAttacker
vector<int> foePlatsDensity;
vector<int> denseFoePlats;
// updTaskForTransporter6
vector<vector<D>> sellDis;
vector<D> makeCost;
vector<D> sellDisTo7;
vector<int> num456;
// 全局区 end

void initGlobalVariables()
{
  // updTaskForAttacker
  if (foePlatsDensity.empty() && !mpFoe.plats.empty())
  {
    const double R = 8; // 参数，工作台密度判断范围
    foePlatsDensity.resize(mpFoe.plats.size());
    for (size_t i = 0; i < foePlatsDensity.size(); ++i)
      for (size_t j = 0; j < foePlatsDensity.size(); ++j)
        if (mpFoe.plats[i].p.dis(mpFoe.plats[j].p) <= R)
          ++foePlatsDensity[i];
    int mx = *std::max_element(foePlatsDensity.begin(), foePlatsDensity.end());
    for (int z = mx; z >= mx - 2; --z)
    {
      for (size_t i = 0; i < foePlatsDensity.size(); ++i)
        if (foePlatsDensity[i] >= z && mpFoe.plats[i].type >= 4)
          denseFoePlats.push_back(i);
      if (int(denseFoePlats.size()) >= 3)
        break;
    }
    if (denseFoePlats.empty())
    {
      for (size_t i = 0; i < foePlatsDensity.size(); ++i)
        if (foePlatsDensity[i] == mx)
          denseFoePlats.push_back(i);
    }

    for (auto &r : mp.robots)
      if (r.isAttacker())
      {
        double mnDis = 1e9;
        for (int type = 1; type <= 3; ++type)
        {
          for (auto &bplat : mp.plats)
            if (bplat.type == type)
            {
              double dis1 = mp.getEvalDis(r.p, bplat.id, 0, 0.43);
              if (dis1 >= 1e9 - 100)
                continue;
              for (int spid : denseFoePlats)
              {
                double dis2 = mpFoe.getEvalDis(bplat.p, spid, 0, 0.53);
                if (dis2 >= 1e9 - 100)
                  continue;
                if (dis1 + dis2 < mnDis)
                {
                  mnDis = dis1 + dis2;
                  r.attackBuy123 = bplat.id;
                }
              }
            }
          if (r.attackBuy123 != -1)
            break;
        }
        if (r.attackBuy123 != -1)
          break;
      }
  }

  // updTaskForTransporter6
  if (sellDis.empty() && !mp.plats.empty())
  {
    int sz = mp.plats.size();
    sellDis.resize(sz, vector<D>(sz));
    for (int i = 0; i < sz; ++i)
      for (int j = 0; j < sz; ++j)
      {
        sellDis[i][j] = 1e9;
        auto &bplat = mp.plats[i];
        auto &splat = mp.plats[j];
        if (splat.needBuy >> bplat.sell & 1)
          sellDis[i][j] = mp.getEvalDis(bplat.p, splat.id, 0, 0.53);
      }

    makeCost.resize(sz);
    for (int i = 0; i < sz; ++i)
    {
      auto &splat = mp.plats[i];
      if (!(splat.type >= 4 && splat.type <= 7))
        continue;
      for (int item = 1; item <= 6; ++item)
        if (splat.needBuy >> item & 1)
        {
          double mn = 1e9;
          for (int j = 0; j < sz; ++j)
          {
            auto &bplat = mp.plats[j];
            if (bplat.sell != item)
              continue;
            mn = min(mn, sellDis[j][i]);
          }
          if (mn >= 1e9 - 100)
          {
            makeCost[i] = 1e9;
            break;
          }
          makeCost[i] += mn;
        }
    }

    sellDisTo7.resize(sz);
    for (int i = 0; i < sz; ++i)
    {
      sellDisTo7[i] = 1e9;
      auto &bplat = mp.plats[i];
      if (!(bplat.type >= 4 && bplat.type <= 6))
        continue;
      for (int j = 0; j < sz; ++j)
      {
        auto &splat = mp.plats[j];
        if (splat.type != 7)
          continue;
        sellDisTo7[i] = min(sellDisTo7[i], sellDis[i][j]);
      }
    }

    // dbg(sellDis[8][4]);
    // dbg(sellDisTo7[8]);
    // exit(0);

    num456.resize(10);
  }
}

void OK() { std::cout << "OK" << std::endl; }
void readOK()
{
  std::string s;
  std::cin >> s;
  myAssert(s == "OK");
}

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
  {
    mp.robots[2].type = 1;
    mp.robots[3].type = 1;
  }
  else if (mp.type == 4)
  {
    for (auto &r : mp.robots)
      r.type = 1;
  }
  else
  {
    if (mp.role == 0)
    {
      // mp.robots[2].type = 1;
      mp.robots[3].type = 1;
    }
    else
    {
      // mp.robots[2].type = 1;
      mp.robots[3].type = 1;
    }
  }
}

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
  mp.init(role, g, true);
  mpFoe.init(role ^ 1, g, false);

  initRobotType();
  Logger::instance().load("logs/log_" + std::to_string(mp.mapId) + ".txt");

  initGlobalVariables();

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
    else if (r.isAttacker() && r.attackBuy123 >= 0 && r.attackBuy123 == r.pid)
    {
      r.item = mp.plats[r.pid].sellItem();
      if (r.item != 0)
      {
        moneyJustLossTF -= BUY_PRICE[r.item];
        moneyJustLossCF -= BUY_PRICE[r.item];
        r.radius = 0.53;
        r.attackBuy123 = -2;
        std::cout << "buy " << r.id << '\n';
      }
    }
}

void updTaskForTransporter6()
{
  // dbg(num456[4], num456[5], num456[6]);
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
          auto &bplat = mp.plats[rob.buy];
          auto &splat = mp.plats[rob.sell];
          if (splat.needBuy == (splat.willBuy | splat.alreadyBuy))
          {
            --num456[splat.type];
          }
          splat.willBuy ^= 1 << bplat.sell;
          --bplat.willSellNum;
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
    int minNum456 = min({num456[4], num456[5], num456[6]});
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
          if (splat.isSafe && splat.canBuy(rob.item))
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
            D edis1 = mp.getEvalDis(rob.p, bplat.id, area, 0.45);
            D dis1 = std::max(edis1,
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

                double baseCof = 1;
                baseCof += __builtin_popcount(splat.willBuy | splat.alreadyBuy) * 10;
                if (bplat.type == 7 && edis1 <= 1) // 防止经过7工作台不卖7
                  baseCof += 50;
                if (bplat.type >= 4 && bplat.type <= 6 && splat.type == 7 && edis1 <= 1)
                  baseCof += 50;
                if (splat.type >= 4 && splat.type <= 6 && num456[splat.type] == minNum456 && sellDisTo7[splat.id] <= 1e9 - 100)
                  baseCof += 6.6;
                if (bplat.type >= 1 && bplat.type <= 3 && splat.type == 9)
                  baseCof -= 8.8;
                if (bplat.type >= 4 && bplat.type <= 6 && splat.type == 9 && num[bplat.type] <= 1 && sellDisTo7[bplat.id] <= 1e9 - 100)
                  continue;
                baseCof -= __builtin_popcount(bplat.willBuy) * 30.0;
                Score *= baseCof;

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
      {
        ++num[splat.type];
        if (splat.type >= 4 && splat.type <= 6 && sellDisTo7[splat.id] <= 1e9 - 100)
          ++num456[splat.type];
      }
      rob.state = 1;
    }
  }

  auto swithTarget = [&](Robot &r)
  {
    if (r.sell != -1)
    {
      auto &splat = mp.plats[r.sell];
      if (splat.visitRobotId == r.id)
        splat.visitRobotId = -1;
      if ((splat.willBuy | splat.alreadyBuy) == splat.needBuy)
      {
        --num[splat.type];
        if (splat.type >= 4 && splat.type <= 6 && sellDisTo7[splat.id] <= 1e9 - 100)
          --num456[splat.type];
      }
      mp.plats[r.sell].willBuy ^= 1 << r.item;
    }
    r.sell = -1;
    double mxScore = -1e9;
    int minNum456 = min({num456[4], num456[5], num456[6]});
    for (auto &splat : mp.plats)
    {
      if (splat.canBuy(r.item))
      {
        double dis = r.p.dis(splat.p);
        double Score = BUY_PRICE[r.item] / dis;

        double baseCof = 1;
        baseCof += __builtin_popcount(splat.willBuy | splat.alreadyBuy) * 10;
        if (splat.type >= 4 && splat.type <= 6 && num456[splat.type] == minNum456 && sellDisTo7[splat.id] <= 1e9 - 100)
          baseCof += 6.6;
        Score *= baseCof;

        int delta = 25;
        double evalTime =
            frameID + dis / MAX_FORWARD_SPEED[mp.role] * FPS + delta;

        if (Score > mxScore && evalTime <= FRAME_LIMIT)
        {
          mxScore = Score;
          r.sell = splat.id;
        }
      }
    }
    if (r.sell != -1)
    {
      auto &splat = mp.plats[r.sell];
      splat.willBuy ^= 1 << r.item;
      if ((splat.willBuy | splat.alreadyBuy) == splat.needBuy)
      {
        ++num[splat.type];
        if (splat.type >= 4 && splat.type <= 6 && sellDisTo7[splat.id] <= 1e9 - 100)
          ++num456[splat.type];
      }
    }
  };

  for (auto &r : robots)
    if (r.isTransporter() && r.state == 2)
    {
      if (r.isStop)
      {
        if (r.sell != -1)
        {
          auto &splat = mp.plats[r.sell];
          if (splat.visitRobotId == r.id)
            splat.visitRobotId = -1;
          if ((splat.willBuy | splat.alreadyBuy) == splat.needBuy)
          {
            --num[splat.type];
            if (splat.type >= 4 && splat.type <= 6 && sellDisTo7[splat.id] <= 1e9 - 100)
              --num456[splat.type];
          }
          mp.plats[r.sell].willBuy ^= 1 << r.item;
        }
      }
      else if (r.sell == -1 || !mp.plats[r.sell].isSafe && !mp.plats[r.sell].rushAble)
        swithTarget(r);
    }

  int tot = 0;
  for (auto &r : robots)
    if (r.isTransporter())
      ++tot;
  for (auto &r : robots)
    if (tot > 1 && frameID > FRAME_LIMIT - 200 && r.state == 0)
    {
      r.type = 1;
      --tot;
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
      double dis = e.pos.dis(r.p);
      if (std::abs(angle) > PI / 2 && dis > 3)
        continue;
      if (dis < mndis)
      {
        mndis = e.pos.dis(r.p);
        r.attackFoeId = id;
      }
    }
  };

  auto &robots = mp.robots;
  for (auto &r : robots)
    if (r.isAttacker())
    {

      if (r.attackBuy123 >= 0)
        continue;

      interfereFoe(r);
      if (r.attackFoeId != -1)
        continue;
      if (frameID % 3000 == 0)
        r.attackPlatId = -1;
      if (r.attackPlatId == -1 && !denseFoePlats.empty())
      {
        std::shuffle(denseFoePlats.begin(), denseFoePlats.end(), Random::instance().rng);
        for (int pid : denseFoePlats)
          if (mpFoe.getEvalDis(r.p, pid, 0, r.radius) <= 1e9 - 100)
          {
            r.attackPlatId = denseFoePlats.back();
            break;
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

bool checkAccessibility(Robot &r, Point nex)
{
  if (r.p.dis(nex) <= 0.1)
    return true;
  Point direction = nex - r.p;
  double robR = r.radius - 0.01; // 参数，影响矩形判断精度
  Rectangle rec = {r.p + direction.rotate(PI / 2) * robR,
                   r.p + direction.rotate(-PI / 2) * robR,
                   nex + direction.rotate(-PI / 2) * robR,
                   nex + direction.rotate(PI / 2) * robR};
  Circle cir = {nex, r.radius};
  double angleDis = normAngle(r.p.angleTo(nex) - r.angle);
  if (angleDis < 0)
    angleDis += 2 * PI;
  int mid = angleDis / PI * 180;
  int range = 40; // 参数，枚举雷达的范围
  for (int j = mid - range; j <= mid + range; ++j)
  {
    double len = r.radar[(j % 360 + 360) % 360];
    assert(len >= r.radius);
    double tmpAngle = r.angle + 1.0 * j / 180 * PI;
    len += 0.1;
    Point obs = r.p + Point(cos(tmpAngle), sin(tmpAngle)) * len;
    if (rec.isIn(obs) || cir.isIn(obs))
      return false;
  }
  return true;
}

void processRadarInfo()
{
  auto &robots = mp.robots;
  std::vector<Foe> fs;
  auto isInMyRob = [&](Pt &p)
  {
    for (auto &r : robots)
      if (std::abs(r.p.dis(p) - r.radius) <= EPS)
        return true;
    return false;
  };
  auto isObst = [&](int i, int j)
  {
    int x = mp.n - 1 - j, y = i;
    return x < 0 || y < 0 || x >= mp.n || y >= mp.n || mp.g[x][y] == '#';
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
  for (size_t i = 0; i < robots.size(); ++i)
  {
    auto &r = robots[i];
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
      for (auto &e : fs)
        if (e.pos.dis(center) <= EPS2)
          isNotIn = false;
      if (isNotIn)
        fs.push_back(Foe(center, Pt(0, 0), radius, 0));
    }
  }
  for (auto &r : robots)
    for (auto &e : fs)
    {
      Point v = e.pos - r.p;
      v = v.unit();
      if (checkAccessibility(r, e.pos - v * (e.radius + r.radius)))
        e.visibleRobotMask |= 1 << r.id;
    }

  // 每帧最大移动距离为7/50=0.14
  for (auto &e : fs)
    for (auto &eOld : mp.foes)
      if (eOld.pos.dis(e.pos) <= 0.2)
        e.velocity = (e.pos - eOld.pos) * FPS;
  mp.foes = fs;

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

void updHisInfo()
{
  for (auto &plat : mp.plats)
    plat.hisDist.push_back(-1);
  for (auto &e : mp.foes)
    for (auto &plat : mp.plats)
    {
      double edis = mp.getEvalDis(e.pos, plat.id, 0, e.radius);
      if (plat.hisDist.back() < 0 || edis < plat.hisDist.back())
        plat.hisDist.back() = edis;
    }
  for (auto &plat : mp.plats)
    plat.updSafety();

  for (auto &r : mp.robots)
  {
    r.hisVelocity.push_back(r.v.len());
    r.updIsStop();
  }
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

    std::cout << frameID << '\n';
    processRadarInfo();
    updHisInfo();
    sellItem();
    updTask();
    buyItem();
    moveRobots();
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