#include "Map.hpp"
#include "constants.hpp"

void Map::getRobotAndPlatformInfo()
{
  using namespace std;
  auto st = [&](vector<int> vc)
  {
    int ret = 0;
    for (int val : vc)
      ret += 1 << val;
    return ret;
  };
  const vector<int> needTimes = {-1, 50, 50, 50, 500, 500, 500, 1000, 1, 1};
  const vector<int> needBuys = {-1, 0,
                                0, 0,
                                st({1, 2}), st({1, 3}),
                                st({2, 3}), st({4, 5, 6}),
                                st({7}), st({1, 2, 3, 4, 5, 6, 7})};
  const vector<int> sells = {-1, 1, 2, 3, 4, 5, 6, 7, 0, 0};

  mapToPlatID.assign(n, std::vector<int>(n, -1));
  for (int i = 0; i < n; ++i)
  {
    string &s = g[i];
    for (int j = 0; j < n; ++j)
    {
      double x = j * 0.5 + 0.25;
      double y = (n - 1 - i) * 0.5 + 0.25;
      if (!role)
      {
        if (s[j] == 'A')
        {
          int id = robots.size();
          robots.emplace_back(Robot(id, x, y));
        }
        else if (s[j] >= '1' && s[j] <= '9')
        {
          int id = plats.size();
          mapToPlatID[i][j] = id;
          int type = s[j] - '0';
          plats.emplace_back(Platform(id, type, needTimes[type], needBuys[type],
                                      sells[type], 1, x, y));
        }
      }
      else
      {
        if (s[j] == 'B')
        {
          int id = robots.size();
          robots.emplace_back(Robot(id, x, y));
        }
        else if (s[j] >= 'a' && s[j] <= 'i')
        {
          int id = plats.size();
          mapToPlatID[i][j] = id;
          int type = s[j] - 'a' + 1;
          plats.emplace_back(Platform(id, type, needTimes[type], needBuys[type],
                                      sells[type], 1, x, y));
        }
      }
    }
  }
}

LL Map::getHash()
{
  const int M1 = 998244353, M2 = 1e9 + 9, B1 = 1331, B2 = 131;
  LL h1 = 0, h2 = 0;
  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < n; j++)
    {
      h1 = (h1 * B1 + g[i][j]) % M1;
      h2 = (h2 * B2 + g[i][j]) % M2;
    }
  }
  return h1 << 31 | h2;
}

void Map::getMapId()
{
  const std::vector<LL> MAP_ID = {
      -1,
  };

  LL hash = getHash();
  dbg("hash code:", hash);

  mapId = 0;
  for (int i = 1; i < MAP_ID.size(); i++)
  {
    if (hash == MAP_ID[i])
    {
      mapId = i;
      break;
    }
  }

  dbg("map id:", mapId);
}

void Map::init()
{
  std::string str = "BLUE";
  std::cin >> str;
  if (str == "BLUE")
  {
    role = 0;
  }
  else if (str == "RED")
  {
    role = 1;
  }
  else
  {
    myAssert(false);
  }

  g.resize(n);
  for (int i = 0; i < n; i++)
    std::cin >> g[i];
  getRobotAndPlatformInfo();
  getMapId();
  initNodes();
  buildGraph();
  initEvalDisPP();
}

void Map::initNodes()
{
  mapToNodeID.assign(gn, std::vector<std::vector<int>>(gn));
  for (int i = 0; i < gn; i++)
  {
    for (int j = 0; j < gn; j++)
    {
      addNode(Pt(i * gstep, j * gstep));
    }
  }

  { // init Key Platforms
    platIsKey.resize(plats.size());

    auto check = [&](std::vector<std::string> str, int val) -> void
    {
      int sn = str.size(), sm = str[0].size();
      for (int i = 0; i + sn <= n; i++)
      {
        for (int j = 0; j + sm <= n; j++)
        {
          bool flag = true;
          for (int x = 0; x < sn; x++)
          {
            for (int y = 0; y < sm; y++)
            {
              if (str[x][y] == '#' && g[i + x][j + y] != '#' ||
                  str[x][y] != '#' && g[i + x][j + y] == '#')
              {
                flag = false;
                goto FlagKey;
              }
            }
          }
        FlagKey:
          if (flag)
          {
            for (int x = 0; x < sn; x++)
            {
              for (int y = 0; y < sm; y++)
              {
                int id = mapToPlatID[i + x][j + y];
                if (id >= 0)
                  platIsKey[id] = val;
              }
            }
          }
        }
      }
    };

    std::vector<std::string> ss;
    ss = {"#..#"};
    check(ss, 1);
    ss = {"#", ".", ".", "#"};
    check(ss, 2);
    ss = {"#...", "...#"};
    check(ss, 1);
    std::swap(ss[0], ss[1]);
    check(ss, 1);
    ss = {"#.", "..", "..", ".#"};
    check(ss, 2);
    ss = {".#", "..", "..", "#."};
    check(ss, 2);
  }

  std::vector<D> dirs;
  for (int i = 0; i < 8; i++)
  {
    dirs.push_back(PI / 4 * i);
  }

  auto get = [&](D x)
  { return x / 180.0 * PI; };

  D t = get(29.5);
  dirs.push_back(-PI / 2 + t);
  dirs.push_back(-PI / 2 - t);
  dirs.push_back(PI / 2 + t);
  dirs.push_back(PI / 2 - t);
  dirs.push_back(t);
  dirs.push_back(-t);
  dirs.push_back(PI + t);
  dirs.push_back(PI - t);

  std::vector<Pt> dps;
  for (auto &d : dirs)
    dps.emplace_back(std::cos(d), std::sin(d));

  auto add = [&](Pt p)
  {
    for (Pt dp : dps)
    {
      addNode(p + dp * 0.4);
      addNode(p + dp * 0.35);
      addNode(p + dp * 0.3);
      addNode(p + dp * 0.2);
      addNode(p + dp * 0.15);
      addNode(p + dp * 0.10);
      addNode(p + dp * 0.05);
    }
  };

  for (auto &plat : plats)
  {
    add(plat.p);
    if (platIsKey[plat.id])
    {
      for (Pt dp : dps)
      {
        addNode(plat.p + dp * 0.65);
      }
    }
  }

  // for (int i = 0; i < n; i++) {
  //   for (int j = 0; j < n; j++) {
  //     if (origin_map[i][j] != '#') {
  //       Pt p(j * 0.5 + 0.25, (n - 1 - i) * 0.5 + 0.25);
  //       int c1 = Obstacle::getNearbyObstaclePoints(origin_map, p,
  //       1).size(); int c2 = Obstacle::getNearbyObstaclePoints(origin_map,
  //       p, 2).size();

  //       if (c2 - c1 >= 4 && c1 <= 2) {
  //         add(p);
  //       }
  //     }
  //   }
  // }
}

void Map::addNode(Pt pos)
{
  static std::set<Pt> s;
  if (pos.x < 0 || pos.y < 0 || pos.x >= 50.0 || pos.y >= 50.0)
    return;
  if (!s.insert(pos).second)
    return;
  mapToNodeID[pos.x / gstep][pos.y / gstep].push_back(nodes.size());
  nodes.emplace_back(pos);
}

std::vector<int> Map::getReachableNeighbors(Pt p, D radius, int d)
{
  std::vector<Pt> os = Obstacle::getNearbyObstaclePoints(g, p, d / 2 + 3);
  for (auto &o0 : os)
  {
    if (Obstacle::checkOS(o0, p, p, radius))
    {
      return {};
    }
  }

  std::vector<int> neighbors;
  int x = p.x / gstep, y = p.y / gstep;
  for (int i = std::max(0, x - d); i <= x + d && i < gn; i++)
  {
    for (int j = std::max(0, y - d); j <= y + d && j < gn; j++)
    {
      for (auto id : mapToNodeID[i][j])
      {
        const Pt &q = nodes[id].p;

        if (p == q)
          continue;

        bool flag = false;
        for (auto &o0 : os)
        {
          if (Obstacle::checkOS(o0, p, q, radius))
          {
            flag = true;
            break;
          }
        }
        if (!flag)
        {
          neighbors.push_back(id);
        }
      }
    }
  }
  return neighbors;
}

std::vector<int> Map::getPlatformNeighbors(int pid)
{
  std::vector<int> s;
  auto &p = plats[pid];
  int x = p.p.x / gstep, y = p.p.y / gstep;
  int d = 2;
  for (int i = std::max(0, x - d); i <= x + d && i < gn; i++)
  {
    for (int j = std::max(0, y - d); j <= y + d && j < gn; j++)
    {
      for (int id : mapToNodeID[i][j])
      {
        if (nodes[id].p.dis(p.p) <= 0.4)
        {
          s.push_back(id);
        }
      }
    }
  }
  return s;
}

std::vector<int> Map::getPlatformNeighbors2(int pid)
{
  D rad = platIsKey[pid] ? 0.7 : 0.4;
  std::vector<int> s;
  auto &p = plats[pid];
  int x = p.p.x / gstep, y = p.p.y / gstep;
  int d = 2;
  for (int i = std::max(0, x - d); i <= x + d && i < gn; i++)
  {
    for (int j = std::max(0, y - d); j <= y + d && j < gn; j++)
    {
      for (int id : mapToNodeID[i][j])
      {
        if (nodes[id].p.dis(p.p) <= rad)
        {
          s.push_back(id);
        }
      }
    }
  }
  return s;
}

bool Map::queryReachability(Pt a, Pt b, D radius)
{
  int ax = n - 1 - a.y * 2, ay = a.x * 2;
  int bx = n - 1 - b.y * 2, by = b.x * 2;

  int lx = std::max(-1, std::min(ax, bx) - 2);
  int rx = std::min(n, std::max(ax, bx) + 2);
  int ly = std::max(-1, std::min(ay, by) - 2);
  int ry = std::min(n, std::max(ay, by) + 2);
  // if ((rx - lx + 1) <= 5 || (ry - ly + 1) <= 5)

  {
    for (int i = lx; i <= rx; i++)
    {
      for (int j = ly; j <= ry; j++)
      {
        if (i < 0 || j < 0 || i == n || j == n || g[i][j] == '#')
        {
          D u = j * 0.5, v = (n - 1 - i) * 0.5;
          if (Obstacle::checkOS(Pt(u, v), a, b, radius))
            return false;
        }
      }
    }
    return true;
  }

  // TODO
  if (rx - lx > ry - ly)
  {
    auto f = [&](int x) -> int
    {
      D y = (n - 1 - x) * 0.5;
      smax(y, std::min(a.y, b.y));
      smin(y, std::max(a.y, b.y));
      D t = (a.x - b.x) / (a.y - b.y) * (y - a.y) + a.x;
      return t * 2;
    };

    for (int i = lx; i <= rx; i++)
    {
      int y = f(i);
      for (int j = std::max(-1, y - 2); j <= y + 2 && j <= n; j++)
      {
        if (i < 0 || j < 0 || i == n || j == n || g[i][j] == '#')
        {
          D u = j * 0.5, v = (n - 1 - i) * 0.5;
          if (Obstacle::checkOS(Pt(u, v), a, b, radius))
            return false;
        }
      }
    }
  }
  else
  {
    auto f = [&](int y) -> int
    {
      D x = 0.5 * y;
      smax(x, std::min(a.x, b.x));
      smin(x, std::max(a.x, b.x));
      D t = (a.y - b.y) / (a.x - b.x) * (x - a.x) + a.y;
      return n - 1 - t * 2;
    };
    for (int j = ly; j <= ry; j++)
    {
      int x = f(j);
      for (int i = std::max(-1, x - 2); i <= x + 2 && i <= n; i++)
      {
        if (i < 0 || j < 0 || i == n || j == n || g[i][j] == '#')
        {
          D u = j * 0.5, v = (n - 1 - i) * 0.5;
          if (Obstacle::checkOS(Pt(u, v), a, b, radius))
            return false;
        }
      }
    }
  }
  return true;
}

Pt Map::getNextTo(Pt cur, int pid, int area, D r,
                  std::vector<std::pair<Pt, Pt>> &segs)
{
  auto &d = distFromPlat[!cmp(r, R[1])];

  D dist = 1e9;
  int t = -1;

  auto ns = getReachableNeighbors(cur, r, 2);
  // std::cerr << ns.size() * segs.size() << "\n";

  for (auto o : ns)
  {
    // std::cerr << "grn " << o << "\n";

    D z = pid >= 0 ? d[pid][area][o].first : 1e9;
    // if (z == 1e9)
    //   continue;
    z += nodes[o].p.dis(cur);

    if (smin(dist, z))
    {
      t = o;
    }
  }

  bool flag = false;

  ns = getReachableNeighbors(cur, r, 4);
  for (auto o : ns)
  {
    Pt u = cur, v = nodes[o].p;
    if (!segs.empty())
    {
      D dis1 = 1e9, dis2 = 1e9;
      D f = 1.5, g = 2.5;
      for (auto &[a, b] : segs)
      {
        if (distSS(a, b, u, v) < f)
        {
          smin(dis1, distPS(v, a, b));
          smin(dis2, distPS(u, a, b));
        }
      }
      if (dis2 <= g)
      {
        D z = (dis2 - dis1) * 1e6;
        if (smin(dist, z))
        {
          t = o;
          flag = true;
        }
      }
    }
  }
  if (t == -1)
    return cur;
  if (pid < 0 || flag)
    return nodes[t].p;

  std::vector<int> path;
  while (t != -1)
  {
    path.push_back(t);
    t = d[pid][area][t].second;
  }

  for (int i = 1; i < path.size(); i++)
  {
    if (!queryReachability(cur, nodes[path[i]].p, r + 0.02))
    {
      return nodes[path[i - 1]].p;
    }
  }

  return nodes[path.back()].p;
}

struct Graph
{
  const std::vector<Map::Node> &nodes;
  int n, m;

  Graph(const std::vector<Map::Node> &nodes)
      : nodes(nodes), n(nodes.size()), deg(nodes.size() + 1) {}

  void addEdge(int u, int v, D w)
  {
    deg[u]++;
    edges.push_back({u, v, w});
  }

  void buildEdges()
  {
    for (int i = 1; i <= n; i++)
    {
      deg[i] += deg[i - 1];
    }
    g.resize(m = edges.size());
    for (auto &[u, v, w] : edges)
    {
      g[--deg[u]] = {v, w};
    }
  }

  std::vector<std::pair<D, int>>
  dijkstra(const std::vector<std::pair<int, D>> &s)
  {
    std::vector<std::pair<D, int>> d(n, {1e9, -1});

    while (!q.empty())
      q.pop();

    for (auto [x, y] : s)
    {
      q.emplace(y, x);
      d[x].first = y;
    }

    while (!q.empty())
    {
      QInfo info = q.top();
      q.pop();
      if (info.d != d[info.x].first)
        continue;
      int x = info.x;
      for (int i = deg[x]; i < deg[x + 1]; i++)
      {
        auto [y, z] = g[i];
        if (y == d[x].second)
          continue;
        if (d[x].second != -1)
        {
          Pt u = nodes[d[x].second].p, v = nodes[x].p, w = nodes[y].p;
          myAssert(!(u == v) && !(u == w) && !(v == w));
          u = v - u, v = w - v;

          D ang = std::acos(dot(u, v) / u.len() / v.len());
          D angCost =
              ang / MAX_ROTATE_SPEED * MAX_FORWARD_SPEED[Map::instance().role];
          if (ang < PI / 6)
          {
            z += angCost * 0.1;
          }
          else if (ang < PI / 4.6)
          {
            z += angCost * 0.7;
          }
          else if (ang < PI / 4)
          {
            z += angCost * 2 + 0.2;
          }
          else
          {
            z += angCost * 2.8 + 0.5;
          }
        }
        if (smin(d[y].first, d[x].first + z))
        {
          d[y].second = x;
          q.emplace(d[y].first, y);
        }
      }
    }
    return d;
  }

  struct QInfo
  {
    D d;
    int x;
    bool operator<(const QInfo &r) const { return d > r.d; }
    QInfo(D d, int x) : d(d), x(x) {}
  };

  struct Edge
  {
    int u, v;
    D w;
  };

  std::vector<std::pair<int, D>> g;
  std::vector<int> deg;
  std::vector<Edge> edges;

  std::priority_queue<QInfo> q;
};

void Map::buildGraph()
{
  for (int k = 0; k < 2; k++)
  {
    Graph g(nodes);
    for (int i = 0; i < nodes.size(); i++)
    {
      for (int j : getReachableNeighbors(nodes[i].p, R[k], 2))
      {
        g.addEdge(j, i, (nodes[i].p - nodes[j].p).len());
      }
    }

    g.buildEdges();

    dbg(g.n, g.m);

    distFromPlat[k].resize(plats.size());

    for (auto &p : plats)
    {
      auto &dist = distFromPlat[k][p.id];

      auto tt = getPlatformNeighbors2(p.id);

      std::vector<std::pair<int, D>> ns;
      for (int o : tt)
      {
        ns.push_back({o, nodes[o].p.dis(p.p) <= 0.4 ? 0.0 : 1e9});
      }
      dist.push_back(g.dijkstra(ns));

      if (platIsKey[p.id])
      {
        dist.resize(5);
        if (platIsKey[p.id] == 1)
        {
          p.validAreas = {3, 4};
        }
        else
        {
          p.validAreas = {1, 2};
        }
        for (int area : p.validAreas)
        {
          std::vector<std::pair<int, D>> newNs;
          for (auto [o, v] : ns)
          {
            if (checkPointArea(area, nodes[o].p - p.p))
              newNs.push_back({o, v == 0 ? 0.0 : 5});
          }
          dist[area] = g.dijkstra(newNs);
        }
      }
      else
      {
        p.validAreas = {0};
      }
    }
  }
}