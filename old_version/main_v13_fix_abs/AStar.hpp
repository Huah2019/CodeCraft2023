// #pragma once
// #include "Map.hpp"

// struct AStarGraph {
//   const Graph &g;
//   AStarGraph(const Graph &g) : g(g) {}

//   struct QInfo {
//     D d, v;
//     int x;
//     bool operator<(const QInfo &r) const { return v > r.v; }
//     QInfo(D d, D v, int x) : d(d), v(v), x(x) {}
//   };

//   std::priority_queue<QInfo> q;
//   std::vector<int> findPath(const DistInfo &s, const std::vector<int> &t, const DistInfo &di) {
//     std::vector<QInfo> d(g.n, QInfo(1e9, 1e9, -1));
//     std::vector<int> vis(g.n);
//     for (auto )
//     while (!q.empty())
//       q.pop();

//     for (auto [x, y] : s) {
//       q.emplace(d[x] = QInfo(y, y + di[x].first, x));
//       vis[x] = true;
//     }

//     while (!q.empty()) {
//       QInfo info = q.top();
//       q.pop();
//       int x = info.x;

//       for (int i = g.deg[x]; i < g.deg[x + 1]; i++) {
//         auto [y, z] = g.g[i];
//         if (d[x].x != -1) {
//           Pt u = g.nodes[d[x].x].p, v = g.nodes[x].p, w = g.nodes[y].p;
//           myAssert(!(u == v) && !(u == w) && !(v == w));
//           u = v - u, v = w - v;

//           D ang = std::acos(dot(u, v) / u.len() / v.len());
//           D angCost = ang / MAX_ROTATE_SPEED * MAX_FORWARD_SPEED[Map::instance().role];
//           if (ang < PI / 6) {
//             z += angCost * 0.1;
//           } else if (ang < PI / 4.6) {
//             z += angCost * 0.7;
//           } else if (ang < PI / 4) {
//             z += angCost * 2 + 0.2;
//           } else {
//             z += angCost * 2.8 + 0.5;
//           }
//         }
//         if (smin(d[y].d, d[x].d + z)) {
//           d[y].x = x;
//           q.emplace(d[y].d, d[y].y);
//         }
//       }
//     }
//   }
// };

// struct AStarMap {

// };