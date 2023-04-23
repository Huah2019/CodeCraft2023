#pragma once
#include "Map.hpp"
#include <cassert>

constexpr D ROBOT_GAP = -0.05;
constexpr D FOE_GAP = -0.01;

// struct AStar {
//   const std::vector<int> &foeNeighbors, &robotNeighbors;
//   const Pt now;
//   const D radius;
//   const D dynamicObstHorizon;
//   AStar(const std::vector<int> &foeNeighbors,
//         const std::vector<int> &robotNeighbors, const Pt &now, D radius,
//         D dynamicObstHorizon)
//       : foeNeighbors(foeNeighbors), robotNeighbors(robotNeighbors), now(now),
//         radius(radius), dynamicObstHorizon(dynamicObstHorizon) {}

//   D calExtraCost(Pt a, Pt b, D d) {
//     D cost = 0;
//     for (int i : foeNeighbors) {
//       auto &f = Map::instance().foes[i];
//       D dis = distPS(f.pos, a, b);
//       D k = dis - f.radius - radius - FOE_GAP;
//       if (k <= 0) {
//         return 1e9;
//       }
//       if (f.pos.dis(a) - f.pos.dis(b) > 0.1)
//         cost += 2.0 / k;
//     }

//     if (d < dynamicObstHorizon) {
//       // Pt v = b - a;
//       // D vl = v.len();
//       // for (int i : robotNeighbors) {
//       //   auto &r = Map::instance().robots[i];
//       //   D proj = std::max(0.0, -dot(r.v, v) / vl);
//       //   D dis = distPS(r.p, a, b);
//       //   D k = dis - r.radius - radius - ROBOT_GAP;

//       //   if (proj <= 1 && r.v.len() < 0.1) continue;
//       //   if (k <= 0) {
//       //     return 1e9;
//       //   }

//       //   // cost += 1.0 / k;
//       // }
//     }

//     return cost;
//   }

//   bool check(Pt a, Pt b, D d) {
//     if (d < dynamicObstHorizon) {
//       for (int i : foeNeighbors) {
//         auto &f = Map::instance().foes[i];
//         if (distPS(f.pos, a, b) <= f.radius + radius + FOE_GAP) {
//           return false;
//         }
//       }
//       // Pt v = b - a;
//       // D vl = v.len();
//       // for (int i : robotNeighbors) {
//       //   auto &r = Map::instance().robots[i];
//       //   D proj = std::max(0.0, -dot(r.v, v) / vl);
//       //   D dis = distPS(r.p, a, b);
//       //   D k = dis - r.radius - radius - ROBOT_GAP;
//       //   if (proj <= 1 && r.v.len() < 0.1) continue;
//       //   if (k <= 0) {
//       //     return false;
//       //   }
//       // }
//     }

//     return true;
//   }

//   struct QInfo {
//     D d, v;
//     int x;
//     bool operator<(const QInfo &r) const { return v > r.v; }
//     QInfo(D d, D v, int x) : d(d), v(v), x(x) {}
//   };

//   template <class F1, class F2>
//   std::vector<int> findPath(const Graph &g, const DistInfo &s, F1
//   &&isTerminal,
//                             F2 &&calEvalCost, D distLimit) {
//     std::vector<QInfo> d(g.n, QInfo(1e18, 1e18, -1));
//     std::vector<D> dist(g.n);
//     std::vector<bool> vis(g.n);
//     std::priority_queue<QInfo> q;

//     for (auto [y, x] : s) {
//       d[x] = QInfo(y, y + calEvalCost(x), -1);
//       dist[x] = y;
//       q.emplace(d[x].d, d[x].v, x);
//     }

//     while (!q.empty()) {
//       QInfo info = q.top();
//       q.pop();
//       int x = info.x;

//       if (vis[x])
//         continue;
//       vis[x] = true;

//       for (int i = g.deg[x]; i < g.deg[x + 1]; i++) {
//         auto [y, z] = g.g[i];
//         if (y == d[x].x)
//           continue;

//         D extraCost = calExtraCost(g.nodes[x].p, g.nodes[y].p, dist[x]);

//         if (extraCost >= 1e9)
//           continue;

//         // if (d[x].x != -1) {
//         //   Pt u = g.nodes[d[x].x].p, v = g.nodes[x].p, w = g.nodes[y].p;
//         //   // myAssert(!(u == v) && !(u == w) && !(v == w));
//         //   u = v - u, v = w - v;

//         //   D ang = std::acos(dot(u, v) / u.len() / v.len());
//         //   extraCost += calAngCost(ang, Map::instance().role);
//         // }
//         if (smin(d[y].d, d[x].d + z + extraCost)) {
//           dist[y] = dist[x] + z;
//           d[y].v = d[y].d + calEvalCost(y);
//           d[y].x = x;
//           q.emplace(d[y].d, d[y].v, y);
//         }
//         if (isTerminal(y) || dist[y] > distLimit) {
//           std::vector<int> path;
//           for (; y != -1; y = d[y].x) {
//             path.push_back(y);
//           }
//           std::reverse(path.begin(), path.end());
//           return path;
//         }
//       }
//     }

//     return {};
//   }

//   std::vector<int> path;
//   Pt getNextToPlat(int pid, int area) {
//     auto &mp = Map::instance();
//     auto &mpFoe = Map::instanceFoe();

//     path = mp.getPath(now, pid, area, radius);

//     if (path.empty())
//       return now;

//     bool flag = true;
//     Pt cur = now;
//     D dist = 0;
//     for (int i : path) {
//       Pt p = mp.nodes[i].p;
//       dist += cur.dis(p);
//       cur = p;
//       if (!check(cur, p, dist)) {
//         flag = false;
//         break;
//       }
//     }

//     if (!flag) {
//       auto ns = mpFoe.getReachableNeighbors(now, radius, 2);

//       DistInfo di;
//       for (int x : ns) {
//         if (check(now, mpFoe.nodes[x].p, 0)) {
//         di.emplace_back(mpFoe.nodes[x].p.dis(now), x);
//         }
//         // di.emplace_back(0, x);
//       }

//       int k = !cmp(radius, R[1]);

//       const Pt platPos = mp.plats[pid].p;
//       auto &g = *mpFoe.graph[k];
//       auto &evalDistInfo = mp.distFromPlat[k][pid][area];
//       auto isTerminal = [&](int x) -> bool {
//         return g.nodes[x].p.dis2(platPos) <= 0.16;
//       };
//       auto calEvalCost = [&](int x) -> D { return evalDistInfo[x].first; };
//       path = findPath(g, di, isTerminal, calEvalCost, 1e9);
//       if (path.empty())
//         return now;

//       for (int i = 0; i < path.size(); i++) {
//         if (g.nodes[path[i]].p.dis2(platPos) <= 1) {
//           path.resize(i + 1);
//           break;
//         }
//       }

//       for (;;) {
//         int x = evalDistInfo[path.back()].second;
//         if (x == -1)
//           break;
//         path.push_back(x);
//       }
//     }

//     for (int i = 1; i < path.size(); i++) {
//       Pt p = mp.nodes[path[i]].p;
//       if (!check(now, p, now.dis(p)) ||
//           !mp.queryReachability(now, p, radius + 0.02)) {
//         return mp.nodes[path[i - 1]].p;
//       }
//     }
//     return mp.nodes[path.back()].p;
//     // int i = 0;
//     // while (check(now, g.nodes[path[i]].p, 0) &&
//     //        mpFoe.queryReachability(now, g.nodes[path[i]].p, radius +
//     0.02))
//     //   i++;

//     // return g.nodes[path[i - 1]].p;
//   }
// };

struct AStar
{
  const Robot &robot;
  D dynamicObstHorizon;
  AStar(const Robot &r, D dynamicObstHorizon)
      : robot(r), dynamicObstHorizon((dynamicObstHorizon)) {}

  D calExtraCost(Pt a, Pt b, D d)
  {
    D cost = 0;
    for (int i : robot.foeNeigbors)
    {
      auto &f = Map::instance().foes[i];
      D dis = distPS(f.pos, a, b);
      D k = dis - f.radius - robot.radius - FOE_GAP;
      if (k <= 0)
      {
        return 1e9;
      }
      if (f.pos.dis(a) - f.pos.dis(b) > 0.1)
        cost += 2.0 / k;
    }

    if (d < dynamicObstHorizon)
    {
      // Pt v = b - a;
      // D vl = v.len();
      // for (int i : robotNeighbors) {
      //   auto &r = Map::instance().robots[i];
      //   D proj = std::max(0.0, -dot(r.v, v) / vl);
      //   D dis = distPS(r.p, a, b);
      //   D k = dis - r.radius - radius - ROBOT_GAP;

      //   if (proj <= 1 && r.v.len() < 0.1) continue;
      //   if (k <= 0) {
      //     return 1e9;
      //   }

      //   // cost += 1.0 / k;
      // }
    }

    return cost;
  }

  bool check(Pt a, Pt b, D d)
  {
    for (int i : robot.foeNeigbors)
    {
      auto &f = Map::instance().foes[i];
      if (distPS(f.pos, a, b) <= f.radius + robot.radius + FOE_GAP)
      {
        return false;
      }
    }
    if (d < dynamicObstHorizon)
    {

      // Pt v = b - a;
      // D vl = v.len();
      // for (int i : robotNeighbors) {
      //   auto &r = Map::instance().robots[i];
      //   D proj = std::max(0.0, -dot(r.v, v) / vl);
      //   D dis = distPS(r.p, a, b);
      //   D k = dis - r.radius - radius - ROBOT_GAP;
      //   if (proj <= 1 && r.v.len() < 0.1) continue;
      //   if (k <= 0) {
      //     return false;
      //   }
      // }
    }

    return true;
  }

  struct QInfo
  {
    D d, v;
    int x;
    bool operator<(const QInfo &r) const { return v > r.v; }
    QInfo(D d, D v, int x) : d(d), v(v), x(x) {}
  };

  template <class F1, class F2>
  std::vector<int> findPath(const Graph &g, const DistInfo &s, F1 &&isTerminal,
                            F2 &&calEvalCost, D distLimit)
  {
    auto &q = SpecialHeap<D>::instance(robot.id);
    q.clear();

    auto &val = q.val;
    auto &dist = q.info;
    auto &par = q.par;

    for (auto [y, x] : s)
    {
      dist[x][0] = dist[x][1] = y;
      val[x] = y + calEvalCost(x);
      par[x] = -1;
      q.insert(x);
    }

    while (!q.empty())
    {
      int x = q.top();
      q.pop();

      if (isTerminal(x) || dist[x][0] > distLimit)
      {
        std::vector<int> path;
        for (; x != -1; x = par[x])
        {
          path.push_back(x);
        }
        std::reverse(path.begin(), path.end());
        return path;
      }

      bool debug = false;
      Pt pp = Map::instance().nodes[x].p;
      extern int frameID;
      if (abs(pp.x - 30.5) <= 0.01 && frameID == 3400)
      {
        debug = true;
        dbg(pp, g.deg[x], g.deg[x + 1]);
      }

      for (int i = g.deg[x]; i < g.deg[x + 1]; i++)
      {
        auto [y, z] = g.g[i];
        if (y == par[x])
          continue;

        D extraCost = calExtraCost(g.nodes[x].p, g.nodes[y].p, dist[x][0]);

        if (debug)
        {
          dbg(pp, Map::instance().nodes[y].p, extraCost);
        }

        if (extraCost >= 1e9)
          continue;

        // if (par[x] != -1) {
        //   Pt u = g.nodes[par[x]].p, v = g.nodes[x].p, w = g.nodes[y].p;
        //   // myAssert(!(u == v) && !(u == w) && !(v == w));
        //   u = v - u, v = w - v;

        //   D ang = std::acos(dot(u, v) / u.len() / v.len());
        //   extraCost += calAngCost(ang, Map::instance().role);
        // }

        D eval = dist[x][1] + z + extraCost + calEvalCost(y);
        if (debug)
        {
          dbg(pp, Map::instance().nodes[y].p, extraCost, dist[x][1], calEvalCost(y));
        }
        if (q.isOpen(y))
        {
          dist[y][0] = dist[x][0] + z;
          dist[y][1] = dist[x][1] + z + extraCost;
          val[y] = eval;
          par[y] = x;
          q.insert(y);
        }
        else if (smin(val[y], eval))
        {
          dist[y][0] = dist[x][0] + z;
          dist[y][1] = dist[x][1] + z + extraCost;
          par[y] = x;
          q.decrease(y);
        }
      }
    }

    extern int frameID;
    if (frameID >= 3400 && Map::instance().role && robot.id == 2)
    {
      int a[GN][GN];
      memset(a, 0, sizeof a);
      for (int i = 0; i < q.t; i++)
      {
        int x = q.s[i];
        Pt p = g.nodes[x].p;
        int u = p.x / GSTEP, v = p.y / GSTEP;
        a[GN - v][u] = 1;
      }

      extern std::vector<std::vector<int>> newGrid;

      for (int i = 0; i < GN; i++)
      {
        std::string o;
        for (int j = 0; j < GN; j++)
        {
          if (newGrid[i][j] == -1)
          {
            o += '#';
          }
          else
          {
            o += ".X"[a[i][j]];
          }
        }
        Logger::instance().write(o);
      }

      for (int i = 0; i < GN; i++)
      {
        std::string o;
        for (int j = 0; j < GN; j++)
        {
          if (newGrid[i][j] == -1)
          {
            o += '#';
          }
          else if (newGrid[i][j] == 0)
          {
            o += 'X';
          }
          else
          {
            o += '.';
          }
        }
        Logger::instance().write(o);
      }
      exit(0);
    }
    return {};
    // std::vector<QInfo> d(g.n, QInfo(1e18, 1e18, -1));
    // std::vector<D> dist(g.n);
    // std::vector<bool> vis(g.n);
    // std::priority_queue<QInfo> q;

    // for (auto [y, x] : s) {
    //   d[x] = QInfo(y, y + calEvalCost(x), -1);
    //   dist[x] = y;
    //   q.emplace(d[x].d, d[x].v, x);
    // }

    // while (!q.empty()) {
    //   QInfo info = q.top();
    //   q.pop();
    //   int x = info.x;

    //   if (vis[x])
    //     continue;
    //   vis[x] = true;

    //   for (int i = g.deg[x]; i < g.deg[x + 1]; i++) {
    //     auto [y, z] = g.g[i];
    //     if (y == d[x].x)
    //       continue;

    //     D extraCost = calExtraCost(g.nodes[x].p, g.nodes[y].p, dist[x]);

    //     if (extraCost >= 1e9)
    //       continue;

    //     // if (d[x].x != -1) {
    //     //   Pt u = g.nodes[d[x].x].p, v = g.nodes[x].p, w = g.nodes[y].p;
    //     //   // myAssert(!(u == v) && !(u == w) && !(v == w));
    //     //   u = v - u, v = w - v;

    //     //   D ang = std::acos(dot(u, v) / u.len() / v.len());
    //     //   extraCost += calAngCost(ang, Map::instance().role);
    //     // }
    //     if (smin(d[y].d, d[x].d + z + extraCost)) {
    //       dist[y] = dist[x] + z;
    //       d[y].v = d[y].d + calEvalCost(y);
    //       d[y].x = x;
    //       q.emplace(d[y].d, d[y].v, y);
    //     }
    //     if (isTerminal(y) || dist[y] > distLimit) {
    //       std::vector<int> path;
    //       for (; y != -1; y = d[y].x) {
    //         path.push_back(y);
    //       }
    //       std::reverse(path.begin(), path.end());
    //       return path;
    //     }
    //   }
    // }
    // return {};
  }

  std::vector<int> path;
  std::string pathType;
  Pt getNextToPlat(int pid, int area)
  {
    auto &mp = Map::instance();
    auto &mpFoe = Map::instanceFoe();

    path = mp.getPath(robot.p, pid, area, robot.radius);

    if (path.empty())
      return robot.p;

    bool flag = true;
    Pt cur = robot.p;
    D dist = 0;
    for (int i : path)
    {
      Pt p = mp.nodes[i].p;
      dist += cur.dis(p);
      cur = p;
      if (!check(cur, p, dist))
      {
        flag = false;
        break;
      }
    }

    if (!flag)
    {
      auto ns = mpFoe.getReachableNeighbors(robot.p, robot.radius,
                                            GET_NEXT_TO_NEIGHBOR_RANGE);

      DistInfo di;
      for (int x : ns)
      {
        if (check(robot.p, mpFoe.nodes[x].p, 0))
        {
          di.emplace_back(mpFoe.nodes[x].p.dis(robot.p), x);
        }
        // di.emplace_back(0, x);
      }

      int k = !cmp(robot.radius, R[1]);

      const Pt platPos = mp.plats[pid].p;
      auto &g = *mpFoe.graph[k];
      auto &evalDistInfo = mp.distFromPlat[k][pid][area];
      auto isTerminal = [&](int x) -> bool
      {
        return g.nodes[x].p.dis2(platPos) <= 0.16;
      };
      auto calEvalCost = [&](int x) -> D
      {
        // return g.nodes[x].p.dis(platPos) * 3;
        return evalDistInfo[x].first * 10;
      };
      path = findPath(g, di, isTerminal, calEvalCost, 1e9);
      if (path.empty())
      {
        // extern int frameID;
        // if (frameID >= 3400 && mp.role && robot.id == 2) {
        //   Logger::instance().write("!!!", di.size(), ns.size());
        //   for (int x : ns) {
        //     Logger::instance().write("!ns", mpFoe.nodes[x].p);
        //   }

        // }
        return robot.p;
      }

      for (int i = 0; i < path.size(); i++)
      {
        if (g.nodes[path[i]].p.dis2(platPos) <= 1)
        {
          path.resize(i + 1);
          break;
        }
      }

      for (;;)
      {
        int x = evalDistInfo[path.back()].second;
        if (x == -1)
          break;
        path.push_back(x);
      }
      pathType = "astar_find";
    }
    else
    {
      pathType = "astar_dijkstra_ok";
    }

    for (int i = 1; i < path.size(); i++)
    {
      Pt p = mp.nodes[path[i]].p;
      if (!check(robot.p, p, robot.p.dis(p)) ||
          !mp.queryReachability(robot.p, p, robot.radius + 0.02))
      {
        return mp.nodes[path[i - 1]].p;
      }
    }
    return mp.nodes[path.back()].p;
    // int i = 0;
    // while (check(now, g.nodes[path[i]].p, 0) &&
    //        mpFoe.queryReachability(now, g.nodes[path[i]].p, radius + 0.02))
    //   i++;

    // return g.nodes[path[i - 1]].p;
  }
};
// #.........
// ....###...
// ..........
// ..........
// ..........
// ....######
// #######...
// ..X.X.X...
// ..........
// ..........
// ..........
// ........X.X.X