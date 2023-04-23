#include "NewMap.hpp"
#include "Tools.hpp"
#include <cctype>

struct Graph {
  const std::vector<Pt> &ps; // 每个点的位置
  int n, m;

  Graph(const std::vector<Pt> &ps) : ps(ps), n(ps.size()), deg(ps.size() + 1) {}

  void addEdge(int u, int v, D w) {
    deg[u]++;
    edges.push_back({u, v, w});
  }

  void buildEdges() {
    for (int i = 1; i <= n; i++) {
      deg[i] += deg[i - 1];
    }
    g.resize(m = edges.size());
    // std::sort(edges.begin(), edges.end(),
    //           [&](const Edge &i, const Edge &j) { return i.v > j.v; });
    for (auto &[u, v, w] : edges) {
      g[--deg[u]] = {v, w};
    }
  }

  std::vector<std::pair<D, int>> dijkstra(const std::vector<std::pair<int, D>> &s,
                                          D angleDistWeight) {
    std::vector<std::pair<D, int>> d(n, {1e9, -1});

    while (!q.empty())
      q.pop();

    for (auto [x, y] : s) {
      q.emplace(y, x);
      d[x].first = y;
    }

    while (!q.empty()) {
      QInfo info = q.top();
      q.pop();
      if (info.d != d[info.x].first)
        continue;
      int x = info.x;
      for (int i = deg[x]; i < deg[x + 1]; i++) {
        auto [y, z] = g[i];
        if (y == d[x].second)
          continue;
        if (d[x].second != -1) {
          Pt u = ps[d[x].second], v = ps[x], w = ps[y];
          myAssert(!(u == v) && !(u == w) && !(v == w));
          u = v - u, v = w - v;

          D ang = std::acos(dot(u, v) / u.len() / v.len());

          if (ang > PI / 6)
            z += ang / MAX_ROTATE_SPEED * MAX_FORWARD_SPEED * angleDistWeight;
        }
        if (smin(d[y].first, d[x].first + z)) {
          d[y].second = x;
          q.emplace(d[y].first, y);
        }
      }
    }
    return d;
  }

private:
  struct QInfo {
    D d;
    int x;
    bool operator<(const QInfo &r) const { return d > r.d; }
    QInfo(D d, int x) : d(d), x(x) {}
  };

  struct Edge {
    int u, v;
    D w;
  };

  std::vector<std::pair<int, D>> g;
  std::vector<int> deg;
  std::vector<Edge> edges;

  std::priority_queue<QInfo> q;
};

void NewMap::getRobotAndPlatformInfo() {
  using namespace std;
  auto st = [&](vector<int> vc) {
    int ret = 0;
    for (int val : vc)
      ret += 1 << val;
    return ret;
  };
  const vector<int> need_times = {-1, 50, 50, 50, 500, 500, 500, 1000, 1, 1};
  const vector<int> need_buys = {-1,         0,
                                 0,          0,
                                 st({1, 2}), st({1, 3}),
                                 st({2, 3}), st({4, 5, 6}),
                                 st({7}),    st({1, 2, 3, 4, 5, 6, 7})};
  const vector<int> sells = {-1, 1, 2, 3, 4, 5, 6, 7, 0, 0};

  mapToPlatID.assign(n, std::vector<int>(n, -1));
  for (int i = 0; i < n; ++i) {
    string &s = g[i];
    for (int j = 0; j < n; ++j) {
      double x = j * 0.5 + 0.25;
      double y = (n - 1 - i) * 0.5 + 0.25;
      if (s[j] == 'A') {
        int id = robots.size();
        robots.emplace_back(Robot(id, x, y));
      } else if (s[j] >= '1' && s[j] <= '9') {
        int id = plats.size();
        mapToPlatID[i][j] = id;
        int type = s[j] - '0';
        plats.emplace_back(Platform(id, type, need_times[type], need_buys[type],
                                    sells[type], 1, x, y));
      } else
        assert(s[j] == '.' || s[j] == '#');
    }
  }
}

std::vector<int> NewMap::getReachableNeighbors(Pt p, D radius, int d) {
  std::vector<Pt> os =
      Obstacle::getNearbyObstaclePoints(origin_map, p, d / 2 + 3);
  for (auto &o0 : os) {
    if (Obstacle::checkOS(o0, p, p, radius)) {
      return {};
    }
  }

  std::vector<int> neighbors;
  int x = p.x / gstep, y = p.y / gstep;
  for (int i = std::max(0, x - d); i <= x + d && i < gn; i++) {
    for (int j = std::max(0, y - d); j <= y + d && j < gn; j++) {
      for (auto id : ids[i][j]) {
        const Pt &q = ps[id];

        if (p == q)
          continue;

        bool flag = false;
        for (auto &o0 : os) {
          if (Obstacle::checkOS(o0, p, q, radius)) {
            flag = true;
            break;
          }
        }
        if (!flag) {
          neighbors.push_back(id);
        }
      }
    }
  }

  return neighbors;
}

void NewMap::buildGraph() {

  

  // for (auto &p : plats) {
  //   if (isKey[p.id]) {
  //     dbg("key", p.id, p.p.x, p.p.y, p.type);
  //   }
  // }

  for (int k = 0; k < 2; k++) {
    Graph g(ps);
    for (int i = 0; i < ps.size(); i++) {
      for (int j : getReachableNeighbors(ps[i], R[k], 2)) {
        g.addEdge(j, i, (ps[i] - ps[j]).len());
      }
    }

    g.buildEdges();

    // dbg(g.n, g.m);

    distFromPlat[k].resize(plats.size());

    for (auto &p : plats) {
      D weight = 3;
      if (map_id == 2) {
        weight = 2;
      }
      
      auto &dist = distFromPlat[k][p.id];

      auto tt = getPlatformNeighbors2(p.id);

      std::vector<std::pair<int, D>> ns;
      for (int o : tt) {
        ns.push_back({o, ps[o].dis(p.p) <= 0.4 ? 0.0 : 1e9});
      } 
      dist.push_back(g.dijkstra(ns, weight));

      if (isKey[p.id]) {
        dist.resize(5);
        if (isKey[p.id] == 1) {
          p.validAreas = {3, 4};
        } else {
          p.validAreas = {1, 2};
        }
        for (int area : p.validAreas) {
          std::vector<std::pair<int, D>> newNs;
          for (auto [o, v] : ns) {
            if (checkPointArea(area, ps[o] - p.p))
              newNs.push_back({o, v == 0 ? 0.0 : 5});
          }

          // dbg("!!!", area, newNs.size());
          dist[area] = g.dijkstra(newNs, weight);
        }
        // if (k == 0) continue;
        //   for (int i = 0; i < gn; i++) {
        //     for (int j = 0; j < gn; j++) {
        //       if ((i & 1) && (j & 1)) {
                
        //         int id = j * gn + gn - 1 - i;
        //         int x = dist[3][id].first;
        //         if (x == 1e9) {
        //           fprintf(stderr, " ##");
        //         } else {
        //           fprintf(stderr, "%3d", x);
        //         }
              
        //       }
        //     }
        //     fprintf(stderr, "\n");
        //   }
        // exit(0);
      } else {
        p.validAreas = {0};
      }
    }
  }
}

bool NewMap::queryReachability(Pt a, Pt b, D radius) {
  int ax = MAP_SIZE - 1 - a.y * 2, ay = a.x * 2;
  int bx = MAP_SIZE - 1 - b.y * 2, by = b.x * 2;

  int lx = std::max(-1, std::min(ax, bx) - 2);
  int rx = std::min(MAP_SIZE, std::max(ax, bx) + 2);
  int ly = std::max(-1, std::min(ay, by) - 2);
  int ry = std::min(MAP_SIZE, std::max(ay, by) + 2);
  // if ((rx - lx + 1) <= 5 || (ry - ly + 1) <= 5)

  {
    for (int i = lx; i <= rx; i++) {
      for (int j = ly; j <= ry; j++) {
        if (i < 0 || j < 0 || i == MAP_SIZE || j == MAP_SIZE ||
            origin_map[i][j] == '#') {
          D u = j * 0.5, v = (MAP_SIZE - 1 - i) * 0.5;
          if (Obstacle::checkOS(Pt(u, v), a, b, radius))
            return false;
        }
      }
    }
    return true;
  }

  // TODO
  if (rx - lx > ry - ly) {
    auto f = [&](int x) -> int {
      D y = (MAP_SIZE - 1 - x) * 0.5;
      smax(y, std::min(a.y, b.y));
      smin(y, std::max(a.y, b.y));
      D t = (a.x - b.x) / (a.y - b.y) * (y - a.y) + a.x;
      return t * 2;
    };

    for (int i = lx; i <= rx; i++) {
      int y = f(i);
      for (int j = std::max(-1, y - 2); j <= y + 2 && j <= MAP_SIZE; j++) {
        if (i < 0 || j < 0 || i == MAP_SIZE || j == MAP_SIZE ||
            origin_map[i][j] == '#') {
          D u = j * 0.5, v = (MAP_SIZE - 1 - i) * 0.5;
          if (Obstacle::checkOS(Pt(u, v), a, b, radius))
            return false;
        }
      }
    }
  } else {
    auto f = [&](int y) -> int {
      D x = 0.5 * y;
      smax(x, std::min(a.x, b.x));
      smin(x, std::max(a.x, b.x));
      D t = (a.y - b.y) / (a.x - b.x) * (x - a.x) + a.y;
      return MAP_SIZE - 1 - t * 2;
    };
    for (int j = ly; j <= ry; j++) {
      int x = f(j);
      for (int i = std::max(-1, x - 2); i <= x + 2 && i <= MAP_SIZE; i++) {
        if (i < 0 || j < 0 || i == MAP_SIZE || j == MAP_SIZE ||
            origin_map[i][j] == '#') {
          D u = j * 0.5, v = (MAP_SIZE - 1 - i) * 0.5;
          if (Obstacle::checkOS(Pt(u, v), a, b, radius))
            return false;
        }
      }
    }
  }
  return true;
}

Pt NewMap::getNextTo(Pt cur, int pid, int area, D r, std::vector<Segment> &segs) {
  // std::cerr << pid << " " << distFromPlat[!cmp(r, R[1])].size() << "\n";
  // myAssert(pid < distFromPlat[!cmp(r, R[1])].size());
  // if (pid < 0) return cur;
  // return cur;

  auto &d = distFromPlat[!cmp(r, R[1])];

  D dist = 1e9;
  int t = -1;

  auto ns = getReachableNeighbors(cur, r, 2);
  // std::cerr << ns.size() * segs.size() << "\n";

  for (auto o : ns) {
    // std::cerr << "grn " << o << "\n";

    D z = pid >= 0 ? d[pid][area][o].first : 1e9;
    // if (z == 1e9)
    //   continue;
    z += ps[o].dis(cur);

    if (smin(dist, z)) {
      t = o;
    }
  }

  bool flag = false;

  // if (map_id != 1) {
  ns = getReachableNeighbors(cur, r, 5);
  for (auto o : ns) {
    Segment ss = {cur, ps[o]};
    if (!segs.empty()) {
      D dis1 = 1e9, dis2 = 1e9;
      D f = 1.3, g = 2.5;
      // if (map_id != 1)
        // f = 2, g = 4;
        // f = 1.5, g = 2.5;
      f = 2, g = 4;
      for (auto &s : segs)
        if (s.dis(ss) < f) {
          smin(dis1, s.dis(ps[o]));
          smin(dis2, s.dis(cur));
        }
      if (dis2 <= g) {
        D z = (dis2 - dis1) * 1e6;
        if (smin(dist, z)) {
          t = o;
          flag = true;
        }
      }
    }
  }
  // }
  if (t == -1)
    return cur;
  if (pid < 0 || flag)
    return ps[t];

  // return ps[t];

  std::vector<int> path;
  while (t != -1) {
    path.push_back(t);
    t = d[pid][area][t].second;
  }

  for (int i = 1; i < path.size(); i++) {
    if (!queryReachability(cur, ps[path[i]], r)) {
      return ps[path[i - 1]];
    }
  }

  return ps[path.back()];
}