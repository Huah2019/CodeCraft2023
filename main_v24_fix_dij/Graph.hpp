#pragma once
#include "constants.hpp"
#include "geo.hpp"

struct Node {
  Pt p;

  Node(Pt p) : p(p) {}
};

struct Edge {
  int u, v;
  D w;
};

using DistInfo = std::vector<std::pair<D, int>>;

struct Graph {
  const std::vector<Node> &nodes;
  int n, m;

  Graph(const std::vector<Node> &nodes)
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

  DistInfo dijkstra(const std::vector<std::pair<int, D>> &s, int role) {
    DistInfo d(n, {1e9, -1});

    while (!q.empty())
      q.pop();

    for (auto [x, y] : s) {
      q.emplace(y, x);
      d[x].first = y;
    }

    std::vector<bool> vis(n);
    std::vector<int> xs;
    while (!q.empty()) {
      int x = q.top().x;
      q.pop();
      if (vis[x])
        continue;
      vis[x] = true;
      xs.push_back(x);
      for (int i = deg[x]; i < deg[x + 1]; i++) {
        auto [y, z] = g[i];
        if (vis[y]) continue;
        if (smin(d[y].first, d[x].first + z)) {
          d[y].second = x;
          q.emplace(d[y].first, y);
        }
      }
    }

    std::vector<D> angCost(n);
    for (int x : xs) {
      int p = d[x].second;
      if (p != -1) {
        angCost[x] += angCost[p];
        int q = d[p].second;
        if (q != -1) {
          Pt u = nodes[x].p, v = nodes[p].p, w = nodes[q].p;
          u = v - u, v = w - v;
          angCost[x] += calAngCost(dot(u, v) / u.len() / v.len(), role);
        }
      }
      d[x].first += angCost[x];
    }
    return d;
  }

  struct QInfo {
    D d;
    int x;
    bool operator<(const QInfo &r) const { return d > r.d; }
    QInfo(D d, int x) : d(d), x(x) {}
  };

  std::vector<std::pair<int, D>> g;
  std::vector<int> deg;
  std::vector<Edge> edges;

  std::priority_queue<QInfo> q;
};
