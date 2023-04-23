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

// 全局区 begin
// updTaskForAttacker
vector<int> foePlatsDensity;
vector<int> denseFoePlats;
// updTaskForTransporter6
vector<vector<D>> sellDis;
vector<D> minSellDis;
vector<D> makeCost;
vector<D> sellDisTo7;
int num456[10];
bool canMake7; // 是否存在生产7并卖出的生产线
// 敌人同款信息
vector<D> minSellDisFoe;
vector<vector<D>> sellDisFoe;
vector<D> makeCostFoe;
vector<D> sellDisTo7Foe;
bool canMake7Foe;
vector<int> foePlatsOnBeltline7[10]; // 敌人7号生产线上的工作台计数
// 全局区 end

auto &mp = Map::instance();
auto &mpFoe = Map::instanceFoe();

void initGlobalVariables1()
{
  // updTaskForTransporter6
  if (sellDis.empty() && !mp.plats.empty())
  {
    int sz = mp.plats.size();
    sellDis.resize(sz, vector<D>(sz));
    minSellDis.resize(sz);
    for (int i = 0; i < sz; ++i)
    {
      minSellDis[i] = 1e9;
      for (int j = 0; j < sz; ++j)
      {
        sellDis[i][j] = 1e9;
        auto &bplat = mp.plats[i];
        auto &splat = mp.plats[j];
        if (splat.needBuy >> bplat.sell & 1)
          sellDis[i][j] = mp.getEvalDis(bplat.p, splat.id, 0, 0.53);
        minSellDis[i] = min(minSellDis[i], sellDis[i][j]);
      }
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
      if (makeCost[i] < 1e9 - 100 && splat.type == 7 && minSellDis[i] < 1e9 - 100)
      {
        canMake7 = true;
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
  }
  // dbg("sellDis 2 10", sellDis[2][10], minSellDis[2]);
  // dbg("canMake7", canMake7);

  // 敌人同款信息
  if (sellDisFoe.empty() && !mpFoe.plats.empty())
  {
    int sz = mpFoe.plats.size();
    sellDisFoe.resize(sz, vector<D>(sz));
    minSellDisFoe.resize(sz);
    for (int i = 0; i < sz; ++i)
    {
      minSellDisFoe[i] = 1e9;
      for (int j = 0; j < sz; ++j)
      {
        sellDisFoe[i][j] = 1e9;
        auto &bplat = mpFoe.plats[i];
        auto &splat = mpFoe.plats[j];
        if (splat.needBuy >> bplat.sell & 1)
          sellDisFoe[i][j] = mpFoe.getEvalDis(bplat.p, splat.id, 0, 0.53);
        minSellDisFoe[i] = min(minSellDisFoe[i], sellDisFoe[i][j]);
      }
    }

    makeCostFoe.resize(sz);
    for (int i = 0; i < sz; ++i)
    {
      auto &splat = mpFoe.plats[i];
      if (!(splat.type >= 4 && splat.type <= 7))
        continue;
      for (int item = 1; item <= 6; ++item)
        if (splat.needBuy >> item & 1)
        {
          double mn = 1e9;
          for (int j = 0; j < sz; ++j)
          {
            auto &bplat = mpFoe.plats[j];
            if (bplat.sell != item)
              continue;
            mn = min(mn, sellDisFoe[j][i]);
          }
          if (mn >= 1e9 - 100)
          {
            makeCostFoe[i] = 1e9;
            break;
          }
          makeCostFoe[i] += mn;
        }
      if (makeCostFoe[i] < 1e9 - 100 && splat.type == 7 && minSellDisFoe[i] < 1e9 - 100)
      {
        canMake7Foe = true;
      }
    }

    sellDisTo7Foe.resize(sz);
    for (int i = 0; i < sz; ++i)
    {
      sellDisTo7Foe[i] = 1e9;
      auto &bplat = mpFoe.plats[i];
      if (!(bplat.type >= 4 && bplat.type <= 6))
        continue;
      for (int j = 0; j < sz; ++j)
      {
        auto &splat = mpFoe.plats[j];
        if (splat.type != 7)
          continue;
        sellDisTo7Foe[i] = min(sellDisTo7Foe[i], sellDisFoe[i][j]);
      }
    }

    if (canMake7Foe)
    {
      for (auto &p : mpFoe.plats)
      {
        bool ok = false;
        if (p.type == 7 && minSellDisFoe[p.id] < 1e9 - 100)
          ok = true;
        if (p.type >= 4 && p.type <= 6 && sellDisTo7Foe[p.id] < 1e9 - 100)
          ok = true;
        if (p.type >= 1 && p.type <= 3)
        {
          for (int j = 0; j < sz; ++j)
            if (sellDisTo7Foe[j] < 1e9 - 100 && sellDisFoe[p.id][j] < 1e9 - 100)
            {
              ok = true;
              break;
            }
        }
        if (ok)
          foePlatsOnBeltline7[p.type].push_back(p.id);
      }
    }
  }
  // dbg("canMake7Foe", canMake7Foe);
}

void initGlobalVariables2()
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
      if (r.isAttacker() && r.attackMode == 0)
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
  int maxAttackNum = 0, minAttackNum = 0;
  if (mp.type == 1)
  {
    minAttackNum = 1;
    maxAttackNum = 1;
  }
  else if (mp.type == 2)
  {
    minAttackNum = 1;
    maxAttackNum = 1;
  }
  else if (mp.type == 3)
  {
    minAttackNum = 2;
    maxAttackNum = 2;
  }
  else if (mp.type == 4)
  {
    if (mp.role == 0)
      minAttackNum = maxAttackNum = 4;
    else
      minAttackNum = maxAttackNum = 0;
  }
  else
  {
    minAttackNum = 0;
    maxAttackNum = 1; // default
  }

  vector<int> types(7);
  iota(types.begin(), types.end(), 1);
  stable_sort(types.begin(), types.end(), [&](int x, int y)
              { return foePlatsOnBeltline7[x].size() < foePlatsOnBeltline7[y].size(); });

  // string info;
  // for (int i = 1; i <= 7; ++i)
  //   info += to_string(foePlatsOnBeltline7[i].size()) + " ";
  // dbg(info);

  int attackRobotNum = 0;
  for (int type : types)
  {
    int n = mp.robots.size(), m = foePlatsOnBeltline7[type].size();
    if (m == 0)
      continue;
    if (m > maxAttackNum)
      break;

    vector<vector<double>> disTable(n, vector<double>(m));
    for (int i = 0; i < n; ++i)
      for (int j = 0; j < m; ++j)
        disTable[i][j] = mpFoe.getEvalDis(mp.robots[i].p, foePlatsOnBeltline7[type][j], 0, mp.robots[i].radius);
    // for (int i = 0; i < n; ++i)
    // {
    //   string info;
    //   for (int j = 0; j < m; ++j)
    //     info += to_string(disTable[i][j]) + " ";
    //   dbg(info);
    // }

    vector<vector<double>> dp(n + 1, vector<double>(1 << m, 1e9));
    vector<vector<int>> lastAddPlat(n + 1, vector<int>(1 << m, -1));
    dp[0][0] = 0;
    for (int i = 0; i < n; ++i)
      for (int j = 0; j < 1 << m; ++j)
      {
        if (dp[i + 1][j] > dp[i][j])
        {
          dp[i + 1][j] = dp[i][j];
          lastAddPlat[i + 1][j] = -1;
        }
        if (dp[i][j] < 1e9 - 100)
        {
          // dbg("S: ", i, j, dp[i][j]);
          for (int k = 0; k < m; ++k)
            if (!(j >> k & 1) && disTable[i][k] < 1e9 - 100)
            {
              double cost = dp[i][j] + disTable[i][k];
              if (dp[i + 1][j ^ (1 << k)] > cost)
              {
                dp[i + 1][j ^ (1 << k)] = cost;
                lastAddPlat[i + 1][j ^ (1 << k)] = k;
              }
            }
        }
      }
    // dbg("DP: ", type, n, m, dp[n][(1 << m) - 1]);
    if (dp[n][(1 << m) - 1] < 1e9 - 100)
    {
      // dbg("OKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKK");
      int st = (1 << m) - 1;
      for (int i = n; i >= 1; --i)
      {
        int nexst = st;
        if (lastAddPlat[i][st] != -1)
        {
          mp.robots[i - 1].type = 1;
          mp.robots[i - 1].attackPlatId = foePlatsOnBeltline7[type][lastAddPlat[i][st]];
          mp.robots[i - 1].attackMode = 1;
          // dbg("??????: ", i - 1, foePlatsOnBeltline7[type][lastAddPlat[i][st]]);
          nexst ^= 1 << lastAddPlat[i][st];
        }
        st = nexst;
      }
      attackRobotNum = m;
      break;
    }
  }

  if (mp.type == 2)
  {
    for (auto &r : mp.robots)
      if (r.type != 1)
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
        {
          r.type = 1;
          ++attackRobotNum;
        }
      }
  }
  else
  {
    int cur = mp.robots.size() - 1;
    while (attackRobotNum < minAttackNum)
    {
      while (cur >= 0 && mp.robots[cur].type == 1)
        --cur;
      if (cur < 0)
        break;
      mp.robots[cur].type = 1;
      ++attackRobotNum;
    }
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
  std::vector<PII> xs;
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
          xs.emplace_back(i, j + 2);
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
          xs.emplace_back(i + 2, j);
        i = k;
      }
    }
  for (auto [x, y] : xs)
    newGrid[x][y] = 0;

  auto cal = [&](int len)
  {
    auto checkrow = [&](int r, int c1, int c2)
    {
      if (grid[r][c1] != -1 || grid[r][c2] != -1)
        return false;
      for (int c = c1 + 1; c < c2; ++c)
        if (grid[r][c] != 0)
          return false;
      return true;
    };
    auto checkcol = [&](int c, int r1, int r2)
    {
      if (grid[r1][c] != -1 || grid[r2][c] != -1)
        return false;
      for (int r = r1 + 1; r < r2; ++r)
        if (grid[r][c] != 0)
          return false;
      return true;
    };
    auto ok = [&](int r1, int r2, int c1, int c2)
    {
      for (int r = r1; r <= r2; ++r)
        for (int c = c1; c <= c2; ++c)
          if (grid[r][c] != 0)
            return false;
      return true;
    };
    for (int i = 0; i + len - 1 < gn; ++i)
      for (int j = 0; j + len - 1 < gn; ++j)
      {
        if (!ok(i + 1, i + len - 2, j + 1, j + len - 2))
          continue;
        if ((checkrow(i, j, j + len - 1) ||
             checkrow(i + len - 1, j, j + len - 1)) &&
            (ok(i + 1, i + len - 2, j, j) ||
             ok(i + 1, i + len - 2, j + len - 1, j + len - 1)))
        {
          newGrid[i + len / 2][j + len / 2] = 0;
        }
        if ((checkcol(j, i, i + len - 1) ||
             checkcol(j + len - 1, i, i + len - 1)) &&
            (ok(i, i, j + 1, j + len - 2) ||
             ok(i + len - 1, i + len - 1, j + 1, j + len - 2)))
        {
          newGrid[i + len / 2][j + len / 2] = 0;
        }
      }
  };
  cal(5);
  cal(7);
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

  dbg("F0");
  initGlobalVariables1(); // 顺序不能改
  dbg("F1");
  initRobotType(); // 顺序不能改
  dbg("F2");
  initGlobalVariables2(); // 顺序不能改
  dbg("F3");
  Logger::instance().load("log_" + std::to_string(mp.mapId) + "_" +
                          std::to_string(mp.role) + ".txt");

  readOK();

  // for (int i = 0; i < GN; i++)
  // {
  //   std::string o;
  //   for (int j = 0; j < GN; j++)
  //   {
  //     if (newGrid[i][j] == -1)
  //     {
  //       o += '#';
  //     }
  //     else if (newGrid[i][j] == 0)
  //     {
  //       o += 'X';
  //     }
  //     else
  //     {
  //       o += '.';
  //     }
  //   }
  //   Logger::instance().write(o);
  // }

  // if (!mp.role)
  // {
  //   int a[GN][GN];
  //   memset(a, 0, sizeof a);
  //   auto &d = mp.distFromPlat[1][6][0];
  //   for (int i = 0; i < samplPts.size(); i++)
  //   {
  //     if (d[i].first < 1e9)
  //     {
  //       Pt p = samplPts[i];
  //       int u = p.x / GSTEP;
  //       int v = p.y / GSTEP;
  //       a[GN - v][u] = 1;
  //       if (mp.queryReachability(p, Pt(p.x + 0.5, p.y), R[1]))
  //       {
  //         a[GN - v][u] = 2;
  //       }
  //     }
  //   }
  //   for (int i = 0; i < GN; i++)
  //   {
  //     std::string o;
  //     for (int j = 0; j < GN; j++)
  //     {
  //       if (newGrid[i][j] == -1)
  //       {
  //         o += '#';
  //       }
  //       else
  //       {
  //         o += ".XYZ"[a[i][j]];
  //       }
  //     }
  //     Logger::instance().write(o);
  //   }
  // }

  OK();

  dbg("map type:", mp.type);

  dbg("init OK, time:", getTime());
}

int frameID, money;

void sellItem()
{
  for (Robot &r : mp.robots)
    if (r.state == 2 && r.sell != -1 && r.pid == r.sell)
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
    if (r.state == 1 && r.buy != -1 && r.pid == r.buy)
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
                            MAX_FORWARD_SPEED[mp.role] * FPS * 1.5 +
                        15;
      if (evalTime > FRAME_LIMIT)
      {
        auto &bplat = mp.plats[r.buy];
        auto &splat = mp.plats[r.sell];
        splat.willBuy ^= 1 << bplat.sell;
        --bplat.willSellNum;

        r.state = 0;
        r.buy = r.sell = -1;
        continue;
      }

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
    else if (r.isAttacker() && r.attackBuy123 >= 0 &&
             r.attackBuy123 == r.pid)
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

                Score *= pow(
                    __builtin_popcount(splat.willBuy | splat.alreadyBuy) + 1,
                    1.5);

                if (bplat.type == 7 && edis1 <= 1) // 防止经过7工作台不卖7
                  Score += 999999;

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

  auto swithTarget = [&](Robot &r)
  {
    if (r.sell != -1)
    {
      auto &splat = mp.plats[r.sell];
      if (splat.visitRobotId == r.id)
        splat.visitRobotId = -1;
      if ((splat.willBuy | splat.alreadyBuy) == splat.needBuy)
        --num[splat.type];
      mp.plats[r.sell].willBuy ^= 1 << r.item;
    }
    r.sell = -1;
    double mxScore = -1e9;
    for (auto &splat : mp.plats)
    {
      if (splat.canBuy(r.item))
      {
        double dis = r.p.dis(splat.p);
        double Score = BUY_PRICE[r.item] / dis;

        Score *=
            pow(__builtin_popcount(splat.willBuy | splat.alreadyBuy) + 1, 1.5);

        if (num[splat.type] < 0)
          Score += 10000;

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
        ++num[splat.type];
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
            --num[splat.type];
          mp.plats[r.sell].willBuy ^= 1 << r.item;
        }
      }
      else if (r.sell == -1 ||
               !mp.plats[r.sell].isSafe && !mp.plats[r.sell].rushAble)
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

void updTaskForTransporter8()
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
}

void updVirtualTask()
{
  auto &robots = mp.robots;
  auto findNewvitualTarge = [&](Robot &r)
  {
    double maxScore = -1e9;
    for (auto &p : mp.plats)
    {
      int z = __builtin_popcount(p.willBuy) + p.willSellNum;
      double Score = mp.getEvalDis(r.p, p.id, 0, r.radius) + z * 99999;
      if (Score > maxScore)
      {
        maxScore = Score;
        r.vitualTargetPID = p.id;
      }
    }
  };

  int tot = 0;
  for (auto &r : robots)
    if (r.isTransporter())
      ++tot;
  for (auto &r : robots)
    if (r.isTransporter() && tot > 1 && r.state == 0)
    {
      if (!mpFoe.plats.empty() && frameID > FRAME_LIMIT - 200)
      {
        r.vitualTargetPID = -1;
        r.type = 1;
        --tot;
      }
      else if (r.vitualTargetPID == -1 || r.vitualTargetPID == r.pid)
      {
        findNewvitualTarge(r);
      }
    }
    else
      r.vitualTargetPID = -1;
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

      if (r.attackMode == 0)
      {
        interfereFoe(r);
        if (r.attackFoeId != -1)
          continue;
        if (frameID % 3000 == 0)
          r.attackPlatId = -1;
        if (r.attackPlatId == -1 && !denseFoePlats.empty())
        {
          std::shuffle(denseFoePlats.begin(), denseFoePlats.end(),
                       Random::instance().rng);
          for (int pid : denseFoePlats)
            if (mpFoe.getEvalDis(r.p, pid, 0, r.radius) <= 1e9 - 100)
            {
              r.attackPlatId = denseFoePlats.back();
              break;
            }
        }
      }
      else
      {
        r.attackJustRotate = false;
        r.attackFoeId = -1;
        if (mp.foes.empty())
          continue;
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
          double disToPlat = mpFoe.getEvalDis(e.pos, r.attackPlatId, 0, e.radius);
          if (dis > 2 || disToPlat > 1.3)
            continue;
          if (dis < mndis)
          {
            mndis = dis;
            r.attackFoeId = id;
          }
        }
        if (r.attackFoeId == -1 && mpFoe.getEvalDis(r.p, r.attackPlatId, 0, r.radius) <= 0.05)
        {
          double mndis = 1e9;
          for (int id = 0; id < mp.foes.size(); ++id)
          {
            auto &e = mp.foes[id];
            if (!(e.visibleRobotMask >> r.id & 1))
              continue;

            // 参数，距离敌方机器人的距离
            double dis = e.pos.dis(r.p);
            if (dis > 2.5)
            {
              if (e.velocity.len() <= 0.001)
                continue;
              dis = max(2.5, dis / e.velocity.len());
            }
            if (dis < mndis)
            {
              mndis = dis;
              r.attackFoeId = id;
              r.attackJustRotate = true;
            }
          }
        }
      }
    }

  // for (auto &r : robots)
  //   if (r.isAttacker())
  //   {
  //     dbg(r.id, r.attackMode, r.attackBuy123, r.attackFoeId, r.attackPlatId);
  //   }
}

void updTask()
{
  // updTaskForTransporter6();
  // updTaskForTransporter7();

  if (canMake7)
  {
    updTaskForTransporter8();
  }
  else
  {
    updTaskForTransporter6();
  }
  updVirtualTask();

  updTaskForAttacker();
}

bool checkAccessibility(const Robot &r, Point nex,
                        int range /*参数：雷达范围*/)
{
  // return true;
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
  for (int j = mid - range; j <= mid + range; ++j)
  {
    double len = r.radar[(j % 360 + 360) % 360];
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
    int x = MAP_SIZE - 1 - j, y = i;
    return x < 0 || y < 0 || x >= MAP_SIZE || y >= MAP_SIZE ||
           mp.g[x][y] == '#';
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
        fs.emplace_back(center, Pt(0, 0), radius, 0);
    }
  }
  // 每帧最大移动距离为7/50=0.14
  for (auto &e : fs)
    for (auto &eOld : mp.foes)
      if (eOld.pos.dis(e.pos) <= 0.2)
        e.velocity = (e.pos - eOld.pos) * FPS;

  for (auto &r : robots)
  {
    for (int i = 0; i < fs.size(); i++)
    {
      auto &e = fs[i];
      Point v = e.pos - r.p;
      D vl = v.len();
      if (checkAccessibility(r, e.pos - v * ((e.radius + r.radius) / vl), 10))
      {
        e.visibleRobotMask |= 1 << r.id;
      }
    }
    r.robotNeighbors.clear();
    for (auto &e : robots)
    {
      if (e.id == r.id)
        continue;
      Pt v = e.p - r.p;
      D vl = v.len();
      if (vl <= ROBOT_NEIGHBOR_HORIZON &&
          checkAccessibility(r, e.p - v * ((e.radius + r.radius) / vl), 2))
      {
        r.robotNeighbors.push_back(e.id);
      }
    }
  }

  mp.foes = fs;
  for (auto &f : mp.aliveFoes)
  {
    f.ttl--;
    f.pos = f.pos + f.velocity * TIME_STEP;
  }
  for (auto &e : mp.foes)
  {
    bool flag = false;
    for (auto &f : mp.aliveFoes)
    {
      if (f.pos.dis(e.pos) < 0.2)
      {
        f = e;
        flag = true;
        break;
      }
    }
    if (!flag)
    {
      mp.aliveFoes.push_back(e);
    }
  }

  for (int i = 0; i < mp.aliveFoes.size(); i++)
  {
    for (auto &p : mp.plats)
    {
      if (mp.aliveFoes[i].pos.dis(p.p) <= IGNORE_DIST && mp.aliveFoes[i].velocity.len2() < IGNORE_SPEED)
      {
        std::swap(mp.aliveFoes[i], mp.aliveFoes.back());
        mp.aliveFoes.pop_back();
        break;
      }
    }
  }

  std::sort(mp.aliveFoes.begin(), mp.aliveFoes.end(), [&](const Foe &u, const Foe &v)
            { return u.ttl > v.ttl; });

  while (mp.aliveFoes.size() > 4 || !mp.aliveFoes.empty() && mp.aliveFoes.back().ttl <= 0)
  {
    mp.aliveFoes.pop_back();
  }

  for (auto &r : robots)
  {
    r.foeNeigbors.clear();
    for (int i = 0; i < mp.aliveFoes.size(); i++)
    {
      if (r.p.dis(mp.aliveFoes[i].pos) <= FOE_NEIGHBOR_HORIZON)
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

D astarTIME, orcaTIME;
int numObstNeighbors, numRobotNeighbors;
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

void fixState()
{
  for (auto &r : mp.robots)
    if (r.isTransporter() && r.item)
    {
      r.state = 2;
      if (r.sell != -1 && !(mp.plats[r.sell].needBuy >> r.item & 1))
        r.buy = r.sell = -1;
      r.fresh();
    }
  for (auto &p : mp.plats)
  {
    if ((p.willBuy | p.alreadyBuy) == p.needBuy && p.type >= 4 && p.type <= 6)
      --num456[p.type];
    p.willBuy = 0;
    p.willSellNum = 0;
    for (auto &r : mp.robots)
      if (r.state == 2 && r.sell == p.id)
        p.willBuy |= 1 << r.item;
      else if (r.state == 1)
      {
        if (r.sell == p.id)
          p.willBuy |= 1 << mp.plats[r.buy].sell;
        else if (r.buy == p.id)
          ++p.willSellNum;
      }
    if ((p.willBuy | p.alreadyBuy) == p.needBuy && p.type >= 4 && p.type <= 6)
      ++num456[p.type];
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

    fixState();

    // static D s1, s2, s3, s4, s5, s6, ss;
    std::cout << frameID << '\n';
    // D t1 = getTime();
    processRadarInfo();
    // D t2 = getTime();
    updHisInfo();
    // D t3 = getTime();
    sellItem();
    // D t4 = getTime();
    updTask();
    // D t5 = getTime();
    buyItem();
    // D t6 = getTime();
    moveRobots();
    // D t7 = getTime();
    // s1 += t2 - t1;
    // s2 += t3 - t2;
    // s3 += t4 - t3;
    // s4 += t5 - t4;
    // s5 += t6 - t5;
    // s6 += t7 - t6;
    // ss += 1;
    // if (ss >= 100) {
    //   dbg(frameID, "radar", s1, "hisinfo", s2, "updTask", s4, "move", s6,
    //   "astar",astarTIME,"orca", orcaTIME,"obst",numObstNeighbors, "rob",
    //   numRobotNeighbors); s1 = s2 = s3 = s4 = s5 = s6 = ss = astarTIME =
    //   orcaTIME = 0; numObstNeighbors = numRobotNeighbors = 0;
    // }
    // if (mp.role)
    // {
    //   Logger::instance().write("--------", frameID, "ALIVE FOES",
    //                            "---------");
    //   for (auto &r : mp.robots)
    //   {
    //     if (r.isAttacker())
    //       continue;
    //     Logger::instance().write("--------", frameID, r.id, r.pathType,
    //                              "---------");
    //     // Logger::instance().write(r.isAttacker(), r.attackBuy123, r.attackPlatId,
    //     //                          r.attackFoeId);
    //     Logger::instance().write(r.buy, r.sell, r.buyPlatArea, r.nextTo);

    //     // if (r.attackFoeId != -1)
    //     //   Logger::instance().write(mp.foes[r.attackFoeId].pos);
    //     Logger::instance().write(r.p);
    //     for (int i : r.path)
    //     {
    //       Logger::instance().write(mp.nodes[i].p);
    //     }
    //   }
    // }
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