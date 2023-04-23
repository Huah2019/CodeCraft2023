#include "Geometry.hpp"
#include "NewMap.hpp"
#include "Platform.hpp"
#include "Robot.hpp"
#include "Tools.hpp"
#include "constants.hpp"
#include "obstacle.hpp"
#include "CollisionAvoid.hpp"
#include <bits/stdc++.h>

using namespace std;
// #define TEST
const int inf = 1e9;
const int map_size = 100;
const int frame_rate = 50;
const int max_frame = 14800;
const double min_line_speed = -2;
const double max_line_speed = 6;
const double min_angle_speed = -pi;
const double max_angle_speed = pi;
const vector<int> buy_price = {0, 3000, 4400, 5800, 15400, 17200, 19200, 76000};
const vector<int> sell_price = {0, 6000, 7600, 9200,
                                22500, 25000, 27500, 105000};
Logger logger;
TimeCounter timeCounter;

double profit_just_loss_tf = 200000;
double profit_just_loss_cf = 200000;
vector<int> sellnum = {0, 0, 0, 0, 0, 0, 0, 0};

vector<long long> hsmaps = {-1, 1124143676554423019ll, 1346556068538860049ll, -1ll, -1ll};

int frameID; // 当前帧编号
int money;   // 当前帧收益

NewMap &mp = NewMap::instance();

void readok()
{
  string s;
  cin >> s;
  assert(s == "OK");
}

void readmap()
{
  mp.init();
  // mp.read(map_size, map_size);
  long long hs = mp.geths();
  for (int i = 1; i < hsmaps.size(); ++i)
    if (hs == hsmaps[i])
      mp.map_id = i;

  std::cerr << "mapid " << mp.map_id << "\n";
}
// ### Platform begin
void Platform::apply_visit(int rid) { apply_visit_list.push_back(rid); }
void Platform::fresh_visit()
{
  if (visit_robot_id == -1)
    return;
  auto &rob = mp.robots[visit_robot_id];
  if (rob.get_target() != id && rob.p.dis(p) >= 1)
    visit_robot_id = -1;
}
void Platform::process_apply_visit_list()
{
  if (visit_robot_id != -1)
    return;
  double mnScore = 1e9;
  for (int rid : apply_visit_list)
  {
    auto &rob = mp.robots[rid];
    double delta = 1;
    double Score = delta;
    if (rob.item)
      Score += sell_price[rob.item] + delta;
    Score = rob.p.dis(p) / Score;
    if (Score < mnScore)
    {
      mnScore = Score;
      visit_robot_id = rid;
    }
  }
  apply_visit_list.clear();
}

bool Platform::can_buy(int item)
{
  if (!active)
    return false;
  if (__builtin_popcount(will_buy) >= 1)
    return false;
  return (need_buy >> item & 1) && !((alreay_buy | will_buy) >> item & 1);
}

bool Platform::is_wait() { return prod_time == 0 && num == capacity; }

void Platform::readState()
{
  int prev_prod_time = prod_time;
  int _type;
  double _x;
  double _y;
  cin >> _type >> _x >> _y;
  assert(_type == type);
  assert(abs(p.x - _x) < 1e-6);
  assert(abs(p.y - _y) < 1e-6);
  cin >> prod_time >> alreay_buy >> num;

  // if (type == 7)
  // {
  //     logger.write("FrameID: " + to_string(frameID));
  //     auto bin = [&](int x)
  //     {
  //         string ans;
  //         for (int i = 7; i >= 1; --i)
  //             if (x >> i & 1)
  //                 ans += '1';
  //             else
  //                 ans += '0';
  //         return ans;
  //     };
  //     logger.write("id: " + to_string(id) + " already_buy " + bin(alreay_buy)
  //     + " will_buy: " + bin(will_buy));
  // }
}

bool Platform::buyitem(int item)
{
  assert(need_buy >> item & 1);
  if (!(alreay_buy >> item & 1))
  {
    alreay_buy |= 1 << item;
    if (will_buy >> item & 1)
      will_buy ^= 1 << item;
    return true;
  }
  return false;
}

int Platform ::sellitem()
{
  if (!num)
    return 0;
  --num;
  --will_sell_num;
  return sell;
}

int Platform ::getnextime()
{
  if (!active)
    return -1;
  // if (will_sell_num)
  //     return -1;
  if (will_sell_num < num)
    return 0;
  if (will_sell_num == num)
    return prod_time;
  return -1;
}
// ### Platform end

// ### Robot begin
bool Robot::in_target()
{
  if (state == 0)
    return p.dis(next_to) <= 0.2;
  else if (state == 1)
    return pid == buy;
  return pid == sell;
}
int Robot::get_target()
{
  if (state == 0)
    return -2;
  if (state == 1)
    return buy;
  return sell;
}
void Robot::fresh()
{
  const double density = 20;
  const double maxF = 250;
  const double maxM = 50;
  radius = item ? 0.53 : 0.45;
  double m = radius * radius * pi * density;
  max_line_acc = maxF / m;
  max_angle_acc = maxM / (m * radius * radius / 2);
}

void Robot::readState()
{
  cin >> pid >> item >> tf >> cf;
  cin >> angle_speed >> v.x >> v.y >> angle;
  cin >> p.x >> p.y;
  // if (id == 1)
  // {
  //     logger.write("Location: " + to_string(id) + ' ' + to_string(p.x) + ' '
  //     + to_string(p.y)); logger.write("item: " + to_string(id) + ' ' +
  //     to_string(item));
  //     // logger.write("Pid: " + to_string(id) + ' ' + to_string(pid));
  // logger.write("id_state_buy_sell_pid: " + to_string(id) + " " +
  // to_string(state) + " " + to_string(mp.plats[buy].type) + " " +
  // to_string(mp.plats[sell].type) + " " + to_string(pid));
  // logger.write("id_state_buy_sell_pid: " + to_string(id) + " " +
  // to_string(state) + " " + to_string(buy) + " " + to_string(sell) + " " +
  // to_string(pid));
  // }
}

void Robot::nextask()
{
  while (!tasks.empty())
  {
    tie(buy, sell) = tasks.front();
    tasks.pop();
    double eval_time =
        p.dis(mp.plats[buy].p) + mp.plats[buy].p.dis(mp.plats[sell].p);
    int delta = 25;
    // if (mp.map_id == 3 || mp.map_id == 4)
    delta = 25;
    eval_time = eval_time / max_line_speed * frame_rate + delta;
    if (eval_time + frameID <= max_frame)
    {
      state = 1;
      break;
    }
  }
}

std::pair<D, D> Robot::getNewArgs(const std::vector<Robot> &rs,
                                  const std::vector<Obstacle> &os)
{
  Pt optV =
      HalfplaneBuilder(*this, rs, os)
          .solve(std::fmin(MAX_FORWARD_SPEED,
                           v.len() + MAX_DELTA_LINE_SPEED_PER_FRAME[item > 0]));
  D angleDist = norm_angle(atan2(optV.y, optV.x) - angle);
  D absAngle = std::fabs(angleDist);
  D maxW = std::min(absAngle / TIME_STEP, MAX_ROTATE_SPEED) * sgn(angleDist);
  D maxV = optV.len();
  D eV, eW;

  eW = maxW;
  if (absAngle > PI / 8)
  {
    eV = maxV * 0.1;
  }
  else
  {
    // eV = maxV * std::cos(absAngle);
    eV = maxV;
    eW = maxW * std::sin(absAngle);
  }

  return {eV, eW};
}
// ### Robot end

void setactive()
{
  if (mp.map_id == 1)
  {
    // vector<int> vc = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 13, 17, 18, 21, 22, 25,
    // 26, 29, 30, 31, 32, 33, 34, 35, 36}; vector<int> vc = {34, 35}; for (int
    // pid : vc)
    //     mp.plats[pid].active = false;
  }
}

void PreProcess()
{
  logger.Load("logs/log_" + to_string(mp.map_id) + ".txt");
  setactive();
}

void ptProcess()
{
  string info = "Sell num for each item:";
  for (int i = 1; i <= 7; ++i)
    info += " " + to_string(sellnum[i]);
  logger.write(info);
  info = "Profit upper bound for each item:";
  for (int i = 1; i <= 7; ++i)
    info += " " + to_string(sellnum[i] * (sell_price[i] - buy_price[i]));
  logger.write(info);

  logger.write("money: " + to_string(money));
  int money_upper_bound = 200000;
  for (int i = 1; i <= 7; ++i)
    money_upper_bound += (sell_price[i] - buy_price[i]) * sellnum[i];
  logger.write("money upper bound: " + to_string(money_upper_bound));
  logger.write("loss money: " + to_string(money_upper_bound - money));
  logger.write("profit rate: " + to_string(1.0 * money / money_upper_bound));
  logger.write("loss rate: " + to_string(1 - 1.0 * money / money_upper_bound));
  logger.write("profit_just_loss_tf: " + to_string(profit_just_loss_tf));
  logger.write("profit_just_loss_cf: " + to_string(profit_just_loss_cf));
}

void updTask5(vector<Robot> &robots)
{
  vector<bool> ok(robots.size());
  int it = robots.size();
  if (frameID > max_frame - 400)
  {
    for (int i = 0; i < robots.size(); ++i)
      if (robots[i].state)
        ok[i] = true;
  }
  else
  {
    for (auto &rob : robots)
    {
      if (rob.state == 1)
      {
        mp.plats[rob.sell].will_buy ^= 1 << mp.plats[rob.buy].sell;
        --mp.plats[rob.buy].will_sell_num;
      }
    }
  }
  vector<int> num(10);
  for (auto &p : mp.plats)
    if (p.type == 7 && (p.alreay_buy | p.will_buy))
    {
      for (int item = 4; item <= 6; ++item)
        if ((p.need_buy >> item & 1) &&
            !((p.alreay_buy | p.will_buy) >> item & 1))
          --num[item];
      if (p.num == 0 && p.prod_time != -1 && p.prod_time <= 500)
        for (int item = 4; item <= 6; ++item)
          --num[item];
    }
    else if (p.type >= 4 && p.type <= 6)
    {
      int x = p.num + (p.prod_time != -1) - p.will_sell_num;
      num[p.type] += x;
    }
  // int mn = *min_element(num.begin(), num.end());
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
        ok[i] = true;
      }
      else
      {
        for (Platform &bplat : mp.plats)
        {
          int nextime = bplat.getnextime();
          if (nextime == -1 || nextime > 50)
            continue;

          // int areas = mp.getPlatAreaNum(bplat.id);
          // for (int area = areas > 1; area < areas; area++) {
          for (int area : bplat.validAreas)
          {
            D dis1 = max(mp.getEvalDis(rob.p, bplat.id, area, 0.45),
                         1.0 * nextime / frame_rate * max_line_speed);
            if (dis1 >= 1e9)
              continue;
            for (Platform &splat : mp.plats)
            {
              if (splat.can_buy(bplat.sell))
              {
                D dis2 = mp.getEvalDisPP(bplat.id, area, splat.id);
                if (dis2 >= 1e9)
                  continue;
                double Score =
                    (sell_price[bplat.sell] - buy_price[bplat.sell]) /
                    (dis1 + dis2);

                Score *=
                    __builtin_popcount(splat.will_buy | splat.alreay_buy) + 1;

                if (bplat.type >= 1 && bplat.type <= 6 && splat.type == 9)
                {
                  Score /= 10;
                }
                if (bplat.type >= 4 && bplat.type <= 6 && splat.type == 9 &&
                    num[bplat.type] <= 1)
                  Score -= 10000;

                if (num[splat.type] < 0)
                  Score += 10000;

                int delta = 25;
                double evalTime = frameID +
                                  (dis1 + dis2) / max_line_speed * frame_rate +
                                  delta;

                // if (bplat.id == 5 && splat.type == 7 && rob.p.y > bplat.p.y)
                // {
                //   dbg(frameID, area, dis1, dis2, Score);
                // }
                if (Score > mxScore && evalTime <= max_frame)
                {
                  mxScore = Score;
                  rid = rob.id;
                  rindex = i;
                  bpid = bplat.id;
                  spid = splat.id;
                  buyPlatArea = area;
                }

                // else if (area >= 1 && rid == rob.id && rindex == i && bpid ==
                // bplat.id && spid == splat.id && (mxScore - Score) / mxScore <
                // EPS) {
                //   buyPlatArea |= 1 << area;
                // }
              }
            }
          }
        }
      }
    }
    // logger.write("rid: " + to_string(rid) + " bpid: " + to_string(bpid) + "
    // spid: " + to_string(spid));
    // std::cerr << "rid " << rid << ' ' << bpid << ' ' << spid << '\n';
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

    auto &bplat = mp.plats[bpid];
    auto &splat = mp.plats[spid];
    rob.buy = bpid;
    rob.sell = spid;
    rob.buyPlatArea = buyPlatArea;
    ++bplat.will_sell_num;
    splat.will_buy |= 1 << bplat.sell;

    if (splat.need_buy == (splat.will_buy | splat.alreay_buy))
      ++num[splat.type];
    rob.state = 1;

    // if (buyPlatArea > 0) {
    //   dbg("!!! ", bpid, mp.plats[spid].type, buyPlatArea);
    // }
  }
}

void updTask6(vector<Robot> &robots)
{
  bool transmode = false;
  // if (mp.map_id == 4)
  //     transmode = true;

  vector<bool> ok(robots.size());
  int it = robots.size();
  if (frameID > max_frame - 400)
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
          mp.plats[rob.sell].will_buy ^= 1 << rob.item;
      }
      else if (rob.state == 1)
      {
        mp.plats[rob.sell].will_buy ^= 1 << mp.plats[rob.buy].sell;
        --mp.plats[rob.buy].will_sell_num;
      }
    }
  }
  vector<int> num(10);
  for (auto &p : mp.plats)
    if (p.type == 7 && (p.alreay_buy | p.will_buy))
    {
      for (int item = 4; item <= 6; ++item)
        if ((p.need_buy >> item & 1) &&
            !((p.alreay_buy | p.will_buy) >> item & 1))
          --num[item];
      if (p.num == 0 && p.prod_time != -1 && p.prod_time <= 500)
        for (int item = 4; item <= 6; ++item)
          --num[item];
    }
    else if (p.type >= 4 && p.type <= 6)
    {
      int x = p.num + (p.prod_time != -1) - p.will_sell_num;
      num[p.type] += x;
    }
  int mn = *min_element(num.begin(), num.end());
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
            if (splat.can_buy(rob.item))
            {
              double dis = rob.p.dis(splat.p);
              double Score = sell_price[rob.item] / dis + 100000;

              int delta = 25;
              double evalTime =
                  frameID + dis / max_line_speed * frame_rate + delta;

              if (Score > mxScore && evalTime <= max_frame)
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
          int nextime = bplat.getnextime();
          if (nextime == -1 || nextime > 50)
            continue;

          for (int area : bplat.validAreas)
          {
            D dis1 = max(mp.getEvalDis(rob.p, bplat.id, area, 0.45),
                         1.0 * nextime / frame_rate * max_line_speed);
            if (dis1 >= 1e9)
              continue;

            for (Platform &splat : mp.plats)
            {
              if (splat.can_buy(bplat.sell))
              {
                // if (!mp.can_to(rob.p, bplat.id, 0.45) ||
                //     !mp.can_to(bplat.p, splat.id, 0.53))
                //   continue;

                // double edis = mp.getEvalDis(rob.p, bplat.id, 0.45);

                // double dis1 =
                //     max(edis, 1.0 * nextime / frame_rate * max_line_speed);
                // double dis2 = mp.getEvalDisPP(
                //     bplat.id,
                //     splat.id); // mp.getEvalDis(bplat.p, splat.id, 0.53);
                D dis2 = mp.getEvalDisPP(bplat.id, area, splat.id);
                if (dis2 >= 1e9)
                  continue;

                double Score =
                    (sell_price[bplat.sell] - buy_price[bplat.sell]) /
                    (dis1 + dis2);

                Score *= pow(
                    __builtin_popcount(splat.will_buy | splat.alreay_buy) + 1,
                    1.5);

                if (bplat.type >= 1 && bplat.type <= 3 && splat.type == 9)
                  Score /= 10;

                if (bplat.type >= 4 && bplat.type <= 6 && splat.type == 9 &&
                    num[bplat.type] <= 1)
                  Score -= 10000;

                Score -= __builtin_popcount(bplat.will_buy) * 1000;

                if (num[splat.type] < 0)
                  Score += 10000;

                // logger.write("SC: " + to_string(Score));
                int delta = 25;
                double evalTime = frameID +
                                  (dis1 + dis2) / max_line_speed * frame_rate +
                                  delta;

                if (Score > mxScore && evalTime <= max_frame)
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
    // logger.write("rid: " + to_string(rid) + " bpid: " + to_string(bpid) + "
    // spid: " + to_string(spid));
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
      splat.will_buy |= 1 << rob.item;
    }
    else
    {
      auto &bplat = mp.plats[bpid];
      auto &splat = mp.plats[spid];
      rob.buy = bpid;
      rob.buyPlatArea = buyPlatArea;
      rob.sell = spid;
      ++bplat.will_sell_num;
      splat.will_buy |= 1 << bplat.sell;
      if (splat.need_buy == (splat.will_buy | splat.alreay_buy))
        ++num[splat.type];
      rob.state = 1;
    }
  }
}
void updTask8(vector<Robot> &robots)
{
  bool transmode = false;
  // if (mp.map_id == 4)
  //     transmode = true;

  vector<bool> ok(robots.size());
  int it = robots.size();
  if (frameID > max_frame - 400)
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
          mp.plats[rob.sell].will_buy ^= 1 << rob.item;
      }
      else if (rob.state == 1)
      {
        mp.plats[rob.sell].will_buy ^= 1 << mp.plats[rob.buy].sell;
        --mp.plats[rob.buy].will_sell_num;
      }
    }
  }
  vector<int> num(10);
  for (auto &p : mp.plats)
    if (p.type == 7)
    {
      for (int item = 4; item <= 6; ++item)
        if ((p.need_buy >> item & 1) &&
            !((p.alreay_buy | p.will_buy) >> item & 1))
          --num[item];
      if (p.num == 0 && p.prod_time != -1 && p.prod_time <= 500)
        for (int item = 4; item <= 6; ++item)
          --num[item];
    }
    else if (p.type >= 4 && p.type <= 6)
    {
      int x = p.num + (p.prod_time != -1) - p.will_sell_num;
      num[p.type] += x;
    }
  int mn = *min_element(num.begin(), num.end());
  while (it--)
  {
    double mxScore = -1e9;
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
            if (splat.can_buy(rob.item))
            {
              double dis = rob.p.dis(splat.p);
              double Score = sell_price[rob.item] / dis + 100000;

              int delta = 25;
              double evalTime =
                  frameID + dis / max_line_speed * frame_rate + delta;

              if (Score > mxScore && evalTime <= max_frame)
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
          int nextime = bplat.getnextime();
          if (nextime == -1 || nextime > 50)
            continue;

          for (int area : bplat.validAreas)
          {
            double edis = mp.getEvalDis(rob.p, bplat.id, area, 0.45);
            D dis1 = max(edis,
                         1.0 * nextime / frame_rate * max_line_speed);

            if (dis1 >= 1e9)
              continue;

            for (Platform &splat : mp.plats)
            {
              if (splat.can_buy(bplat.sell))
              {
                // if (!mp.can_to(rob.p, bplat.id, 0.45) ||
                //     !mp.can_to(bplat.p, splat.id, 0.53))
                //   continue;

                // double edis = mp.getEvalDis(rob.p, bplat.id, 0.45);

                // double dis1 =
                //     max(edis, 1.0 * nextime / frame_rate * max_line_speed);
                // double dis2 = mp.getEvalDisPP(
                //     bplat.id,
                //     splat.id); // mp.getEvalDis(bplat.p, splat.id, 0.53);
                D dis2 = mp.getEvalDisPP(bplat.id, area, splat.id);
                if (dis2 >= 1e9)
                  continue;

                double Score =
                    (sell_price[bplat.sell] - buy_price[bplat.sell]) /
                    (dis1 + dis2);

                Score *= pow(
                    __builtin_popcount(splat.will_buy | splat.alreay_buy) + 1,
                    1.5);

                if (bplat.type >= 4 && bplat.type <= 6 && edis >= 2 && frameID <= max_frame - 1000 && (bplat.will_buy | bplat.alreay_buy) != bplat.need_buy)
                  Score -= 114514;

                // if (bplat.type >= 1 && bplat.type <= 6 && splat.type == 9)
                //   continue;

                if (bplat.type >= 1 && bplat.type <= 6 && splat.type == 9)
                  Score /= 10;

                if (bplat.type >= 4 && bplat.type <= 6 && splat.type == 9 &&
                    num[bplat.type] <= 1)
                  Score -= 10000;

                int z = __builtin_popcount(bplat.will_buy) + bplat.will_sell_num;
                z += __builtin_popcount(splat.will_buy) + splat.will_sell_num;
                Score -= z * 10000000;

                Score -= num[splat.type] * 10000;

                // logger.write("SC: " + to_string(Score));
                int delta = 25;
                double evalTime = frameID +
                                  (dis1 + dis2) / max_line_speed * frame_rate +
                                  delta;

                if (Score > mxScore && evalTime <= max_frame)
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
    // logger.write("rid: " + to_string(rid) + " bpid: " + to_string(bpid) + "
    // spid: " + to_string(spid));
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
      splat.will_buy |= 1 << rob.item;
    }
    else
    {
      auto &bplat = mp.plats[bpid];
      auto &splat = mp.plats[spid];
      rob.buy = bpid;
      rob.buyPlatArea = buyPlatArea;
      rob.sell = spid;
      ++bplat.will_sell_num;
      splat.will_buy |= 1 << bplat.sell;
      if (splat.need_buy == (splat.will_buy | splat.alreay_buy))
        ++num[splat.type];
      rob.state = 1;
    }
  }
}

void updTask7(vector<Robot> &robots)
{
  // map1 96w
  bool transmode = false;
  // if (mp.map_id == 4)
  //     transmode = true;

  vector<bool> ok(robots.size());
  int it = robots.size();
  if (frameID > max_frame - 400)
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
          mp.plats[rob.sell].will_buy ^= 1 << rob.item;
      }
      else if (rob.state == 1)
      {
        mp.plats[rob.sell].will_buy ^= 1 << mp.plats[rob.buy].sell;
        --mp.plats[rob.buy].will_sell_num;
      }
    }
  }
  vector<int> num(10);
  for (auto &p : mp.plats)
    if (p.type == 7 && (p.alreay_buy | p.will_buy))
    {
      for (int item = 4; item <= 6; ++item)
        if ((p.need_buy >> item & 1) &&
            !((p.alreay_buy | p.will_buy) >> item & 1))
          --num[item];
      if (p.num == 0 && p.prod_time != -1 && p.prod_time <= 500)
        for (int item = 4; item <= 6; ++item)
          --num[item];
    }
    else if (p.type >= 4 && p.type <= 6)
    {
      int x = p.num + (p.prod_time != -1) - p.will_sell_num;
      num[p.type] += x;
    }
  int mn = *min_element(num.begin(), num.end());
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
            if (splat.can_buy(rob.item))
            {
              double dis = rob.p.dis(splat.p);
              double Score = sell_price[rob.item] / dis + 100000;

              int delta = 25;
              double evalTime =
                  frameID + dis / max_line_speed * frame_rate + delta;

              if (Score > mxScore && evalTime <= max_frame)
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
          int nextime = bplat.getnextime();
          if (nextime == -1 || nextime > 50)
            continue;
          for (int area : bplat.validAreas)
          {
            D dis1 = max(mp.getEvalDis(rob.p, bplat.id, area, 0.45),
                         1.0 * nextime / frame_rate * max_line_speed);
            if (dis1 >= 1e9)
              continue;

            for (Platform &splat : mp.plats)
            {
              if (splat.can_buy(bplat.sell))
              {
                // if (!mp.can_to(rob.p, bplat.id, 0.45) ||
                //     !mp.can_to(bplat.p, splat.id, 0.53))
                //   continue;

                // double edis = mp.getEvalDis(rob.p, bplat.id, 0.45);

                // double dis1 =
                //     max(edis, 1.0 * nextime / frame_rate * max_line_speed);
                // double dis2 = mp.getEvalDisPP(
                //     bplat.id,
                //     splat.id); // mp.getEvalDis(bplat.p, splat.id, 0.53);

                D dis2 = mp.getEvalDisPP(bplat.id, area, splat.id);
                if (dis2 >= 1e9)
                  continue;

                double Score =
                    (sell_price[bplat.sell] - buy_price[bplat.sell]) /
                    (dis1 + dis2);

                Score *= pow(
                    __builtin_popcount(splat.will_buy | splat.alreay_buy) + 1,
                    1.5);

                if (bplat.type >= 1 && bplat.type <= 6 && splat.type == 9)
                  Score /= 10;

                if (bplat.type >= 4 && bplat.type <= 6 && splat.type == 9 &&
                    num[bplat.type] <= 1)
                  Score -= 10000;

                Score -= __builtin_popcount(bplat.will_buy) * 1000;

                if (num[splat.type] < 0)
                  Score += 10000;

                // logger.write("SC: " + to_string(Score));
                int delta = 25;
                double evalTime = frameID +
                                  (dis1 + dis2) / max_line_speed * frame_rate +
                                  delta;

                if (Score > mxScore && evalTime <= max_frame)
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
    // logger.write("rid: " + to_string(rid) + " bpid: " + to_string(bpid) + "
    // spid: " + to_string(spid));
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
      splat.will_buy |= 1 << rob.item;
    }
    else
    {
      auto &bplat = mp.plats[bpid];
      auto &splat = mp.plats[spid];
      rob.buy = bpid;
      rob.sell = spid;
      rob.buyPlatArea = buyPlatArea;
      ++bplat.will_sell_num;
      splat.will_buy |= 1 << bplat.sell;
      if (splat.need_buy == (splat.will_buy | splat.alreay_buy))
        ++num[splat.type];
      rob.state = 1;
    }
  }
}

void updTask(vector<Robot> &robots)
{
  // updTask1(robots);
  // updTask2(robots);
  // if (mp.map_id == 1)
  //   updTask3(robots);
  // else
  updTask6(robots);
}

void RVO_move(vector<Robot> &robots)
{
  for (Platform &p : mp.plats)
    p.fresh_visit();
  for (Robot &rob : robots)
  {
    rob.fresh();
    rob.wait = false;
    int target = rob.get_target();
    if (target != -2 && mp.getEvalDis(rob) <= 6)
      mp.plats[target].apply_visit(rob.id);
  }
  for (Platform &p : mp.plats)
    p.process_apply_visit_list();

  // if (mp.map_id == 1) {
  //   for (Robot &rob : robots) {
  //     int target = rob.get_target();
  //     if (target != -2 && mp.getEvalDis(rob) <= 6 &&
  //         mp.plats[target].visit_robot_id != rob.id)
  //       rob.wait = true;
  //   }
  // }

  std::vector<Point> pref(robots.size());
  for (auto &r : robots)
  {
    int target = r.get_target();
    double maxSpeed = max_line_speed;
    // if (target != -2) {
    //   double d = mp.getEvalDis(r);
    //   auto segs = mp.aroundObs(mp.plats[target].p);
    //   double g = 1e9;
    //   for (auto &s : segs)
    //     g = min(g, (double)s.dis(mp.plats[target].p));
    //   if (d <= 5 && g <= 1)
    //     maxSpeed *= 0.1;
    // }
    if (r.wait)
      maxSpeed *= 0.05;
    Point p = r.next_to - r.p;
    double d = p.len();
    if (target >= 0)
      d = r.p.dis(mp.plats[target].p);
    if (p.len() <= 1e-8)
      pref[r.id] = Point(0, 0);
    else
      pref[r.id] = p / p.len() * min(maxSpeed, max(0.1, d) / 0.1225);
  }

  for (auto &i : robots)
  {

    i.preferredVelocity = pref[i.id];
  }

  for (auto &r : robots)
  {
    double eV, eW;

    auto os = Obstacle::getNearbyObstacles(mp.origin_map, r.p, 5);
    tie(eV, eW) = r.getNewArgs(robots, os);

    // auto segs = mp.aroundObs(r.p);
    // segs.insert(segs.end(), edgeSegments.begin(), edgeSegments.end());
    // for (auto s : segs)
    // {
    //     Line l1 = {s.a, s.b - s.a};
    //     Line l2 = {r.p, Point(cos(r.angle), sin(r.angle))};
    //     if (!l1.is_par(l2))
    //     {
    //         Point inter = l2.inter(l1);
    //         double angle_dis = norm_angle(r.angle - r.p.angleto(inter));
    //         if (s.is_on(inter) && abs(angle_dis) <= 0.1 && r.p.dis(inter) <=
    //         2)
    //             eV = min(eV, 0.5);
    //     }
    // }

    // if (mp.map_id != 1) {
    double disToRob = 1e9;
    for (auto &rb : robots)
      if (rb.id != r.id)
      {
        double angle_dis = norm_angle(r.angle - r.p.angleto(rb.p));
        if (abs(angle_dis) <= pi / 2)
          disToRob = min(disToRob, r.p.dis(rb.p) - r.radius - rb.radius);
      }

    if (disToRob <= 1.5)
      eV = min(eV, disToRob);
    // }

    cout << "forward " << r.id << " " << eV << "\n";
    cout << "rotate " << r.id << " " << eW << "\n";
  }
}
void move(vector<Robot> &robots)
{
  // DWA_move(robots);
  // demo_move(robots);
  RVO_move(robots);
}

void sellItem(vector<Robot> &robots)
{
  for (Robot &r : robots)
    if (r.state == 2 && r.pid == r.sell)
    {
      bool ok = mp.plats[r.pid].buyitem(r.item);
      if (ok)
      {
        ++sellnum[r.item];
        profit_just_loss_tf += sell_price[r.item] * r.tf;
        profit_just_loss_cf += sell_price[r.item] * r.cf;
        r.radius = 0.45;
        r.item = 0;
        r.state = 0;
        cout << "sell " << r.id << '\n';
      }
    }
}
void buyItem(vector<Robot> &robots)
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
                            max_line_speed * frame_rate +
                        15;
      if (evalTime > max_frame)
        // return;
        continue;

      // vector<Segment> segs = mp.aroundObs(r.p);
      // if (!segs.empty())
      // {
      //     double mndis = 1e9;
      //     for (auto &sg : segs)
      //         mndis = min(mndis, (double)sg.dis(r.p));
      //     if (mndis <= 0.53 + 0.005)
      //         return;
      // }

      r.item = mp.plats[r.pid].sellitem();
      if (r.item != 0)
      {
        profit_just_loss_tf -= buy_price[r.item];
        profit_just_loss_cf -= buy_price[r.item];
        r.radius = 0.53;
        r.state = 2;
        cout << "buy " << r.id << '\n';
        // logger.write("buy: " + to_string(item) + " plat_sell: " +
        // to_string(mp.plats[pid].sell) + " plat_type: " +
        // to_string(mp.plats[pid].type));
      }
    }
}
void getTarget(vector<Robot> &robots)
{
  vector<int> ids(robots.size());
  iota(ids.begin(), ids.end(), 0);
  sort(ids.begin(), ids.end(), [&](int x, int y)
       {
    if (robots[x].item != robots[y].item)
      return robots[x].item > robots[y].item;
    return x < y; });
  vector<Segment> segs;
  for (int id : ids)
  {
    auto &r = robots[id];
    int pid = r.get_target();
    r.next_to = mp.getNextTo(r.p, pid, r.state == 1 ? r.buyPlatArea : 0,
                             r.radius, segs);

    Pt u = r.p, v = r.next_to - u;
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
}

int main()
{
#ifdef TEST
  freopen("../../maps/3.txt", "r", stdin);
#endif

  ios::sync_with_stdio(false);
  cin.tie(0);
  cout.tie(0);

  readmap();
  PreProcess();

#ifdef TEST
  // cout << mp.geths() << '\n';
  // return 0;
  logger.write("PreProcess time: " + to_string(timeCounter.getTime()) + "ms");
  for (int j = mp.gm - 1; j >= 0; --j)
  {
    auto g = [&](int x, int len)
    {
      string s = to_string(x);
      if (x == -1)
        s = "*****";
      while (int(s.size()) < len)
        s = ' ' + s;
      return s;
    };
    string info;
    for (int i = 0; i < mp.gn; ++i)
    {
      int num = mp.dis53[5][i][j];
      if (num == inf)
        num = -1;
      info += g(num, 5) + " ";
    }
    logger.write(info);
  }
  exit(0);
#endif

  readok();
  cout << "OK\n";
  cout.flush();

  int initTime = timeCounter.getTime();
  std::cerr << "init OK, time " << initTime << "\n";

  myAssert(initTime < 5050);

  int preFrameID = 0, jump = 0;
  while (cin >> frameID)
  {
    jump += frameID - preFrameID - 1;
    preFrameID = frameID;

    myAssert(jump < 20);

    // logger.write("Frame: " + to_string(frameID));
    cin >> money;
    int K;
    cin >> K;
    assert(K == mp.plats.size());
    for (Platform &plat : mp.plats)
      plat.readState();
    for (Robot &rob : mp.robots)
      rob.readState();
    readok();

    cout << frameID << '\n';
    // std::cerr << "frame " << frameID << "\n";
    sellItem(mp.robots);
    // std::cerr << "frame " << frameID << " step1\n";
    updTask(mp.robots);
    // std::cerr << "frame " << frameID << " step2\n";
    buyItem(mp.robots);
    // std::cerr << "frame " << frameID << " step3\n";
    getTarget(mp.robots);
    // std::cerr << "frame " << frameID << " step4\n";
    move(mp.robots);
    // std::cerr << "frame " << frameID << " step5\n";
    cout << "OK\n";
    cout.flush();

    // for (auto &r : mp.robots) {
    //   if (r.id)
    //     continue;
    //   dbg(frameID, r.id, r.buy, r.buyPlatArea, r.sell, r.state);
    //   if (r.buy >= 0) {
    //     Pt v = r.p - mp.plats[r.buy].p;
    //     dbg(v.x, v.y);
    //   }
    // }
  }
  ptProcess();
  return 0;
}
