#pragma once
#include "constants.hpp"
#include "geo.hpp"
#include "SpecialHeap.hpp"

struct Node {
  Pt p;
  D cost = 0;
  Node(Pt p) : p(p) {}
};

struct Edge {
  int u, v;
  D w;
};

struct DistInfo {
  D dist;   // 真实距离
  D distEx; // 带额外惩罚的距离
  int par;  // 父亲
  DistInfo(D dist, D distEx, int par) : dist(dist), distEx(distEx), par(par) {}
};
using DistTable = std::vector<DistInfo>;

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

  DistTable dijkstra(const DistTable &s, int role) {
    DistTable dt(n, DistInfo(1e9, 1e9, -1));
    std::vector<D> d(n, 1e9);
    while (!q.empty())
      q.pop();

    for (auto &v : s) {
      int x = v.par;
      dt[x].dist = v.dist;
      dt[x].distEx = v.distEx;
      d[x] = dt[x].distEx + nodes[x].cost;
      q.emplace(d[x], x);
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
        if (vis[y])
          continue;
        D angCost = 0;
        if (dt[x].par != -1) {
          Pt u = nodes[dt[x].par].p, v = nodes[x].p, w = nodes[y].p;
          u = v - u, v = w - v;
          angCost = calAngCost(dot(u, v) / u.len() / v.len(), role);
        }
        if (smin(d[y], d[x] + z + nodes[y].cost + angCost)) {
          dt[y].dist = dt[x].dist + z;
          dt[y].distEx = dt[x].distEx + z + angCost;
          dt[y].par = x;
          q.emplace(d[y], y);
        }
      }
    }

    // std::vector<D> angCost(n);
    // for (int x : xs) {
    //   int p = d[x].second;
    //   if (p != -1) {
    //     angCost[x] += angCost[p];
    //     int q = d[p].second;
    //     if (q != -1) {
    //       Pt u = nodes[x].p, v = nodes[p].p, w = nodes[q].p;
    //       u = v - u, v = w - v;
    //       angCost[x] += calAngCost(dot(u, v) / u.len() / v.len(), role);
    //     }
    //   }
    //   d[x].first += angCost[x];
    // }
    return dt;
  }

  std::vector<std::pair<int, D>> g;
  std::vector<int> deg;
  std::vector<Edge> edges;

  std::priority_queue<QInfo> q;
};
