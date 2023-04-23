#pragma once

#include "Obstacle.hpp"
#include "Platform.hpp"
#include "Robot.hpp"
#include "utils.hpp"

class Map {
private:
  Map() {}

public:
  Map(const Map &) = delete;
  Map &operator=(const Map &) = delete;
  Map(Map &&) = delete;
  Map &operator=(Map &&) = delete;

  static auto &instance() {
    static Map t;
    return t;
  }
  struct Node {
    Pt p;

    Node(Pt p) : p(p) {}
  };

  const int n = 100, gn = n * 2;
  const D gstep = 50.0 / gn;

  int mapId, role;

  std::vector<std::string> g;

  std::vector<Robot> robots;
  std::vector<Platform> plats;
  std::vector<std::vector<int>> mapToPlatID;
  std::vector<int> platIsKey;

  std::vector<Node> nodes;
  std::vector<std::vector<std::vector<int>>> mapToNodeID;

  using DistInfo = std::vector<std::pair<D, int>>;
  std::vector<std::vector<DistInfo>> distFromPlat[2];

  void getRobotAndPlatformInfo();
  LL getHash();
  void getMapId();
  void init();

  void addNode(Pt pos);
  void initNodes();

  std::vector<int> getReachableNeighbors(Pt p, D radius, int d);

  void buildGraph();

  std::vector<int> getPlatformNeighbors(int pid);

  std::vector<int> getPlatformNeighbors2(int pid);

  // 查询 a->b 半径为 radius 是否可达
  bool queryReachability(Pt a, Pt b, D radius);

  Pt getNextTo(Pt, int, int, D, std::vector<std::pair<Pt, Pt>> &);

  D getEvalDis(Pt p, int pid, int area, D r) {
    if (pid < 0)
      return 1e9;

    auto &dis = distFromPlat[!cmp(r, 0.53)][pid][area];
    D ans = 1e9;
    for (int i : getReachableNeighbors(p, r, 2)) {
      smin(ans, dis[i].first + nodes[i].p.dis(p));
    }
    return ans;
  }

  std::vector<std::vector<std::vector<D>>> evalDisPP;
  void initEvalDisPP() {
    evalDisPP.resize(plats.size());
    for (auto &p1 : plats) {
      auto &e = evalDisPP[p1.id];
      e.resize(distFromPlat[0][p1.id].size());
      for (int k : p1.validAreas) {
        e[k].resize(plats.size(), 1e9);
        for (auto &p2 : plats) {
          for (int i : getPlatformNeighbors(p2.id)) {
            smin(e[k][p2.id], distFromPlat[1][p1.id][k][i].first);
          }
        }
      }
    }
  }
  D getEvalDisPP(int p1, int area, int p2) { return evalDisPP[p1][area][p2]; }

  D getEvalDis(Robot &rob) { return getEvalDis(rob.p, rob.pid, 0, rob.radius); }
};

struct Graph {
  const std::vector<Map::Node> &nodes;
  int n, m;

  Graph(const std::vector<Map::Node> &nodes)
      : nodes(nodes), n(nodes.size()), deg(nodes.size() + 1) {}

  void addEdge(int u, int v, D w) {
    deg[u]++;
    edges.push_back({u, v, w});
  }

  void buildEdges() {
    for (int i = 1; i <= n; i++) {
      deg[i] += deg[i - 1];
    }
    g.resize(m = edges.size());
    for (auto &[u, v, w] : edges) {
      g[--deg[u]] = {v, w};
    }
  }

  std::vector<std::pair<D, int>>
  dijkstra(const std::vector<std::pair<int, D>> &s) {
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
          Pt u = nodes[d[x].second].p, v = nodes[x].p, w = nodes[y].p;
          myAssert(!(u == v) && !(u == w) && !(v == w));
          u = v - u, v = w - v;

          D ang = std::acos(dot(u, v) / u.len() / v.len());
          D angCost =
              ang / MAX_ROTATE_SPEED * MAX_FORWARD_SPEED[Map::instance().role];
          if (ang < PI / 6) {
            z += angCost * 0.1;
          } else if (ang < PI / 4.6) {
            z += angCost * 0.7;
          } else if (ang < PI / 4) {
            z += angCost * 2 + 0.2;
          } else {
            z += angCost * 2.8 + 0.5;
          }
        }
        if (smin(d[y].first, d[x].first + z)) {
          d[y].second = x;
          q.emplace(d[y].first, y);
        }
      }
    }
    return d;
  }

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
