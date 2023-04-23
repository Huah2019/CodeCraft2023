#include "CollisionAvoid.hpp"
#include "Map.hpp"
#include "bits/stdc++.h"
#include "utils.hpp"

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

void init()
{
  std::ios::sync_with_stdio(false);
  std::cin.tie(nullptr);
  std::cout << std::fixed << std::setprecision(12);
  std::cerr << std::fixed << std::setprecision(12);

  mp.init();

  Logger::instance().load("logs/log_" + std::to_string(mp.mapId) + ".txt");

  readOK();
  OK();

  dbg("init OK, time:", getTime());
}

int frameID, money;

void sellItem(std::vector<Robot> &robots)
{
  for (Robot &r : robots)
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
void buyItem(std::vector<Robot> &robots)
{
  for (Robot &r : robots)
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
void RVO_move(std::vector<Robot> &robots)
{
  for (Platform &p : mp.plats)
    p.freshVisit();
  for (Robot &rob : robots)
  {
    rob.fresh();
    rob.wait = false;
    int target = rob.getTarget();
    if (target != -2 && mp.getEvalDis(rob) <= 6)
      mp.plats[target].applyVisit(rob.id);
  }
  for (Platform &p : mp.plats)
    p.processApplyVisitList();

  // if (mp.map_id == 1) {
  for (Robot &rob : robots)
  {
    int target = rob.getTarget();
    if (target != -2 && mp.getEvalDis(rob) <= 6 &&
        mp.plats[target].visitRobotId != rob.id)
      rob.wait = true;
  }
  // }

  std::vector<Point> pref(robots.size());
  for (auto &r : robots)
  {
    int target = r.getTarget();
    double maxSpeed = MAX_FORWARD_SPEED[mp.role];
    if (r.wait)
      maxSpeed *= 0.05;
    Point p = r.nextTo - r.p;
    double d = p.len();
    if (target >= 0)
      d = r.p.dis(mp.plats[target].p);
    if (p.len() <= 1e-8)
      pref[r.id] = Point(0, 0);
    else
      pref[r.id] = p / p.len() * std::min(maxSpeed, std::max(0.1, d) / 0.1225);
  }

  for (auto &i : robots)
  {

    i.preferredVelocity = pref[i.id];
  }

  for (auto &r : robots)
  {
    auto os = Obstacle::getNearbyObstacles(mp.g, r.p, 5);
    auto [eV, eW] = getNewArgs(r, robots, os);

    // if (mp.map_id != 1) {
    // double disToRob = 1e9;
    // double mneV = eV;
    // for (auto &rb : robots)
    //   if (rb.id != r.id) {
    //     double angle_dis = normAngle(r.angle - r.p.angleTo(rb.p));
    //     if (abs(angle_dis) <= PI / 3)
    //       if (r.p.dis(rb.p) - r.radius - rb.radius < disToRob) {
    //         disToRob = r.p.dis(rb.p) - r.radius - rb.radius;
    //         mneV = rb.lineSpeed;
    //       }
    //   }
    // if (disToRob <= 0.8)
    //   smin(eV, mneV);
    // }
    // double disToRob = 1e9;
    // for (auto &rb : robots)
    //   if (rb.id != r.id) {
    //     double angle_dis = normAngle(r.angle - r.p.angleTo(rb.p));
    //     if (abs(angle_dis) <= PI / 2)
    //       smin(disToRob, r.p.dis(rb.p) - r.radius - rb.radius - 0.4);
    //   }

    // if (disToRob <= 0.4)
    //   smin(eV, disToRob);

    std::cout << "forward " << r.id << " " << eV << "\n";
    std::cout << "rotate " << r.id << " " << eW << "\n";
  }
}

void demo_move(std::vector<Robot> &robots)
{
  const double MIN_ANGLE = 0.08;
  for (int i = 0; i < robots.size(); ++i)
  {
    Robot &rob = robots[i];
    if (rob.state == 0)
    {
      std::cout << "forward " << ' ' << rob.id << ' ' << 0 << '\n';
      std::cout << "rotate " << ' ' << rob.id << ' ' << 0 << '\n';
      continue;
    }
    double angle_dis = normAngle(rob.p.angleTo(rob.nextTo) - rob.angle);
    double line_dis = rob.p.dis(rob.nextTo);
    const double maxRotateSpeed =
        (angle_dis > 0 ? MAX_ROTATE_SPEED : -MAX_ROTATE_SPEED);
    // const double maxSpeed = min(line_dis / 0.0525, max_line_speed);
    double maxSpeed = MAX_FORWARD_SPEED[mp.role];

    // logger.write("angle_dis", angle_dis);
    if (abs(angle_dis) <
        MIN_ANGLE)
    { // 如果朝向和目标点的夹角很小，直接全速前进
      rob.lineSpeed = maxSpeed;
      rob.angleSpeed = 0;
    }
    else
    {
      if (abs(angle_dis) > PI / 6)
      {
        // 角度太大，全速扭转
        // 速度控制小一点，避免靠近不了工作台
        rob.lineSpeed = maxSpeed * 0.0;
        rob.angleSpeed = maxRotateSpeed;
      }
      else
      {
        rob.lineSpeed =
            maxSpeed * cos(abs(angle_dis)); // 前进速度随角度变小而变大
        rob.angleSpeed =
            maxRotateSpeed * sin(abs(angle_dis)); // 旋转速度随角度变小而变小
      }
      // rob.line_speed = 0;
    }
  }
  for (auto &r1 : robots)
    for (auto &r2 : robots)
    {
      if (r1.id == r2.id)
        continue;
      auto dis = r1.p.dis(r2.p);
      if (dis <= r1.radius + r2.radius + 0.002)
      {
        // 已经碰撞了
        if (r1.p.x < r2.p.x)
        {
          if (r1.angle > 0)
            r1.angleSpeed += MAX_ROTATE_SPEED / 4.5;
          else
            r1.angleSpeed += -MAX_ROTATE_SPEED / 4.5;
        }
        else
        {
          if (r1.angle > 0)
            r1.angleSpeed += -MAX_ROTATE_SPEED / 4.5;
          else
            r1.angleSpeed += MAX_ROTATE_SPEED / 4.5;
        }
      }
    }
  for (Robot &rob : robots)
  {
    std::cout << "forward " << rob.id << ' ' << rob.lineSpeed << '\n';
    std::cout << "rotate " << rob.id << ' ' << rob.angleSpeed << '\n';
  }
}

void moveRobots(std::vector<Robot> &robots)
{
  static std::vector<int> ids;

  if (ids.empty())
  {
    ids.resize(robots.size());
    iota(ids.begin(), ids.end(), 0);
  }

  stable_sort(ids.begin(), ids.end(), [&](int x, int y)
              {
    if (robots[x].item != robots[y].item)
      return robots[x].item > robots[y].item;
    return x < y; });

  for (size_t i = 0; i < ids.size(); ++i)
  {
    auto &r = robots[i];
    bool canMove = false;

    if (canMove == false)
    {
      int j = i;
      while (j)
      {
        std::swap(ids[j], ids[j - 1]);
        --j;
      }
    }
  }

  std::vector<std::pair<Pt, Pt>> segs;
  for (int id : ids)
  {
    auto &r = robots[id];
    int pid = r.getTarget();
    r.nextTo = mp.getNextTo(r.p, pid, r.state == 1 ? r.buyPlatArea : 0,
                            r.radius, segs);

    Pt u = r.p, v = r.nextTo - u;
    if (v.len() < 0.001)
    {
      segs.push_back({u, u});
    }
    else
    {
      segs.push_back(
          {u - v.unit() * r.radius,
           u + v.unit() *
                   (r.radius +
                    std::min(v.len(),
                             (r.v.len() +
                              MAX_DELTA_LINE_SPEED_PER_FRAME[r.item > 0]) /
                                 FPS))});
    }
  }

  RVO_move(robots);
  // demo_move(robots);
}

void updTask6(std::vector<Robot> &robots)
{
  bool transmode = false;
  // if (mp.map_id == 4)
  //     transmode = true;

  std::vector<bool> ok(robots.size());
  int it = robots.size();
  if (frameID > FRAME_LIMIT - 400)
  {
    for (int i = 0; i < robots.size(); ++i)
      if (robots[i].state)
        ok[i] = true;
  }
  else
  {
    for (auto &rob : robots)
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
      if (ok[i])
        continue;
      auto &rob = robots[i];
      if (rob.state == 2)
      {
        if (transmode)
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
        else
          ok[i] = true;
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
        if (robots[i].state == 2)
          assert(ok[i]);
        else if (!ok[i])
        {
          robots[i].state = 0;
          robots[i].buy = robots[i].sell = -1;
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

void updTask7(std::vector<Robot> &robots)
{
  bool transmode = false;
  // if (mp.map_id == 4)
  //     transmode = true;

  std::vector<bool> ok(robots.size());
  int it = robots.size();
  if (frameID > FRAME_LIMIT - 400)
  {
    for (int i = 0; i < robots.size(); ++i)
      if (robots[i].state)
        ok[i] = true;
  }
  else
  {
    for (auto &rob : robots)
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
      if (ok[i])
        continue;
      if (rob.state == 2)
      {
        if (transmode)
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
        else
          ok[i] = true;
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
        if (robots[i].state == 2)
          assert(ok[i]);
        else if (!ok[i])
        {
          robots[i].state = 0;
          robots[i].buy = robots[i].sell = -1;
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

void updTask(std::vector<Robot> &robots) { updTask7(robots); }

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
    sellItem(mp.robots);
    updTask(mp.robots);
    buyItem(mp.robots);
    moveRobots(mp.robots);
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