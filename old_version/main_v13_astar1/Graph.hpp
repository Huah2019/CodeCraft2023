#pragma once
#include "constants.hpp"
#include "geo.hpp"
#include "SpecialHeap.hpp"

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
    auto &q = SpecialHeap<D>::instance(0);
    q.clear();
    auto &d = q.val;
    auto &par = q.par;
    for (auto [x, y] : s) {
      myAssert(q.isOpen(x));
      d[x] = y;
      par[x] = -1;
      q.insert(x);
    }

    while (!q.empty()) {
      int x = q.top();
      q.pop();

      for (int i = deg[x]; i < deg[x + 1]; i++) {
        auto [y, z] = g[i];
        if (y == par[x])
          continue;
        if (par[x] != -1) {
          Pt u = nodes[par[x]].p, v = nodes[x].p, w = nodes[y].p;
          myAssert(!(u == v) && !(u == w) && !(v == w));
          u = v - u, v = w - v;

          D ang = std::acos(dot(u, v) / u.len() / v.len());
          z += calAngCost(ang, role);
        }

        if (q.isOpen(y)) {
          d[y] = d[x] + z;
          par[y] = x;
          q.insert(y);
        } else if (smin(d[y], d[x] + z)) {
          par[y] = x;
          q.decrease(y);
        }
      }
    } 

    DistInfo di(n);
    for (int i = 0; i < n; i++) {
      if (q.isOpen(i)) {
        di[i] = {1e9, -1};
      } else {
        di[i] = {d[i], par[i]};
      }
    }
    return di;

    // DistInfo d(n, {1e9, -1});

    // while (!q.empty())
    //   q.pop();

    // for (auto [x, y] : s) {
    //   q.emplace(y, x);
    //   d[x].first = y;
    // }

    // while (!q.empty()) {
    //   QInfo info = q.top();
    //   q.pop();
    //   if (info.d != d[info.x].first)
    //     continue;
    //   int x = info.x;
    //   for (int i = deg[x]; i < deg[x + 1]; i++) {
    //     auto [y, z] = g[i];
    //     if (y == d[x].second)
    //       continue;
    //     if (d[x].second != -1) {
    //       Pt u = nodes[d[x].second].p, v = nodes[x].p, w = nodes[y].p;
    //       myAssert(!(u == v) && !(u == w) && !(v == w));
    //       u = v - u, v = w - v;

    //       D ang = std::acos(dot(u, v) / u.len() / v.len());
    //       z += calAngCost(ang, role);
    //     }
    //     if (smin(d[y].first, d[x].first + z)) {
    //       d[y].second = x;
    //       q.emplace(d[y].first, y);
    //     }
    //   }
    // }
    // return d;
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
