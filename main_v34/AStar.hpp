#pragma once
#include "Map.hpp"

constexpr D ROBOT_GAP = -0.05;
constexpr D FOE_GAP = -0.25;

struct AStar {
  const Robot &robot;
  AStar(const Robot &r) : robot(r) {}

  bool check(Pt a, Pt b, D radius) {
    for (int i : robot.foeNeigbors) {
      auto &f = Map::instance().aliveFoes[i];
      if (distPS(f.pos, a, b) <= f.radius + radius + FOE_GAP) {
        return false;
      }
    }
    return true;
  }

  D calPenalty(Pt a) {
    D res = 0;
    for (int i : robot.foeNeigbors) {
      res += 10.0 / a.dis2(Map::instance().aliveFoes[i].pos);
    }
    return res;
  }

  template <class F1, class F2>
  std::vector<int> findPath(const Graph &g, const DistTable &s, F1 &&isTerminal,
                            F2 &&calEvalCost) {
    DistTable dt(g.n, DistInfo(1e9, 1e9, -1));
    std::vector<bool> vis(g.n);
    std::priority_queue<QInfo> q;

    for (auto &o : s) {
      int x = o.par;
      dt[x].dist = o.dist + calPenalty(g.nodes[x].p) + g.nodes[x].cost;
      dt[x].distEx = dt[x].dist + calEvalCost(x);
      q.emplace(dt[x].distEx, x);
    }

    std::vector<int> xs;
    while (!q.empty()) {
      int x = q.top().x;
      q.pop();
      if (isTerminal(x)) {
        std::vector<int> path;
        for (; x != -1; x = dt[x].par) {
          path.push_back(x);
        }
        std::reverse(path.begin(), path.end());
        return path;
      }
      if (vis[x])
        continue;
      vis[x] = true;
      xs.push_back(x);

      for (int i = g.deg[x]; i < g.deg[x + 1]; i++) {
        auto [y, z] = g.g[i];
        if (vis[y])
          continue;

        if (!check(g.nodes[x].p, g.nodes[y].p, robot.radius))
          continue;

        z += calPenalty(g.nodes[y].p) + g.nodes[y].cost;

        if (smin(dt[y].distEx, dt[x].dist + z + calEvalCost(y))) {
          dt[y].dist = dt[x].dist + z;
          dt[y].par = x;
          q.emplace(dt[y].distEx, y);
        }
      }
    }

    // for (auto [y, x] : s) {
    //   dist[x][0] = dist[x][1] = y;
    //   val[x] = y + calEvalCost(x);
    //   par[x] = -1;
    //   q.insert(x);
    // }

    [[maybe_unused]] extern int frameID;

    // while (!q.empty()) {
    //   int x = q.top();
    //   q.pop();
    //   if (isTerminal(x) || dist[x][0] > distLimit) {
    //     std::vector<int> path;
    //     for (; x != -1; x = par[x]) {
    //       path.push_back(x);
    //     }
    //     std::reverse(path.begin(), path.end());
    //     return path;
    //   }

    //   for (int i = g.deg[x]; i < g.deg[x + 1]; i++) {
    //     auto [y, z] = g.g[i];
    //     if (q.isClosed(y))
    //       continue;

    //     D extraCost = calExtraCost(g.nodes[x].p, g.nodes[y].p);

    //     if (extraCost >= 1e9)
    //       continue;

    //     // if (par[x] != -1) {
    //     //   Pt u = g.nodes[par[x]].p, v = g.nodes[x].p, w = g.nodes[y].p;
    //     //   // myAssert(!(u == v) && !(u == w) && !(v == w));
    //     //   u = v - u, v = w - v;

    //     //   D ang = std::acos(dot(u, v) / u.len() / v.len());
    //     //   extraCost += calAngCost(ang, Map::instance().role);
    //     // }

    //     D eval = dist[x][1] + z + extraCost + calEvalCost(y);
    //     if (q.isOpen(y)) {
    //       dist[y][0] = dist[x][0] + z;
    //       dist[y][1] = dist[x][1] + z + extraCost;
    //       val[y] = eval;
    //       par[y] = x;
    //       q.insert(y);
    //     } else if (smin(val[y], eval)) {
    //       dist[y][0] = dist[x][0] + z;
    //       dist[y][1] = dist[x][1] + z + extraCost;
    //       par[y] = x;
    //       q.decrease(y);
    //     }
    //   }
    // }

    // if (frameID >= 4100 && Map::instance().role) {
    //   int a[GN][GN];
    //   memset(a, 0, sizeof a);
    //   for (int x : xs) {
    //     Pt p = g.nodes[x].p;
    //     int u = p.x / GSTEP, v = p.y / GSTEP;
    //     a[GN - v][u] = 1;
    //     Pt q = p + Pt(0.5, 0.25);
    //     // if (check(p, q, 0)) {
    //     //   a[GN - v][u] = 3;
    //     // }

    //     // if (GN - v == 172 && u == 122) {
    //     //   Logger::instance().write(dist[x][0], dist[x][1], val[x]);
    //     //   for (int i = g.deg[x]; i < g.deg[x + 1]; i++) {
    //     //     Logger::instance().write(g.nodes[g.g[i].first].p,
    //     // calExtraCost(p, g.nodes[g.g[i].first].p, dist[x][0]));
    //     //   }
    //     // }
    //     // if (Map::instance().queryReachability(p, q, robot.radius)) {
    //     //   a[GN - v][u] = 2;
    //     // }
    //   }

    //   extern std::vector<std::vector<int>> newGrid;

    //   for (int i = 0; i < GN; i++) {
    //     std::string o;
    //     for (int j = 0; j < GN; j++) {
    //       if (newGrid[i][j] == -1) {
    //         o += '#';
    //       } else {
    //         o += ".XYZ"[a[i][j]];
    //       }
    //     }
    //     Logger::instance().write(o);
    //   }
    // }
    return {};
  }

  std::vector<int> path;
  std::string pathType;
  Pt getNextToPlat(int pid, int area) {
    auto &mp = Map::instance();

    path = mp.getPath(robot.p, pid, area, robot.radius);

    if (path.empty())
      return robot.p;

    bool flag = true;
    // Pt cur = robot.p;
    // D dist = 0;
    // for (int i : path) {
    //   Pt p = mp.nodes[i].p;
    //   dist += cur.dis(p);
    //   cur = p;
    //   if (!check(cur, p)) {
    //     flag = false;
    //     break;
    //   }
    // }

    flag = false;

    if (!flag) {
      auto ns = mp.getReachableNeighbors(robot.p, robot.radius,
                                         GET_NEXT_TO_NEIGHBOR_RANGE);

      DistTable dt;
      for (int x : ns) {
        if (check(robot.p, mp.nodes[x].p, robot.radius)) {
          D dist = mp.nodes[x].p.dis(robot.p);
          dt.emplace_back(dist, dist, x);
        }
      }
      const Pt platPos = mp.plats[pid].p;
      auto &g = *mp.graph[robot.item > 0];
      auto &d = mp.getDistTableFromPlat(pid, area, robot.radius);
      auto isTerminal = [&](int x) -> bool {
        return g.nodes[x].p.dis2(platPos) <= 0.4 * 0.4;
      };
      auto calEvalCost = [&](int x) -> D {
        // return g.nodes[x].p.dis(platPos) * 3;
        return d[x].dist * 2;
      };
      path = findPath(g, dt, isTerminal, calEvalCost);
      if (path.empty()) {
        // extern int frameID;
        // if (frameID >= 4100 && Map::instance().role) {
        //   Logger::instance().write(ns.size(), dt.size());
        //   for (int x : ns) {
        //     Logger::instance().write(mp.nodes[x].p, check(robot.p, mp.nodes[x].p, robot.radius));
            
        //   }
        // }
        // if (frameID >= 3400 && mp.role && robot.id == 2) {
        //   Logger::instance().write("!!!", di.size(), ns.size());
        //   for (int x : ns) {
        //     Logger::instance().write("!ns", mp.nodes[x].p);
        //   }

        // }
        return robot.p;
      }

      for (int i = 0; i < path.size(); i++) {
        if (g.nodes[path[i]].p.dis2(platPos) <= 1) {
          path.resize(i + 1);
          break;
        }
      }

      for (;;) {
        int x = d[path.back()].par;
        if (x == -1)
          break;
        path.push_back(x);
      }
      pathType = "astar_find";
    } else {
      pathType = "astar_dijkstra_ok";
    }

    for (int i = 2; i < path.size(); i++) {
      Pt p = mp.nodes[path[i]].p;
      if (!check(robot.p, p, robot.radius) ||
          !mp.queryReachability(robot.p, p, robot.radius + 0.1)) {
        return mp.nodes[path[i - 1]].p;
      }
    }
    return mp.nodes[path.back()].p;
    // int i = 0;
    // while (check(now, g.nodes[path[i]].p, 0) &&
    //        mp.queryReachability(now, g.nodes[path[i]].p, radius + 0.02))
    //   i++;

    // return g.nodes[path[i - 1]].p;
  }
};
