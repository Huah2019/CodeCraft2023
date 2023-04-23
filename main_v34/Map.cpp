#include "Map.hpp"
#include "AStar.hpp"
#include "constants.hpp"

using namespace std;

void Map::getRobotAndPlatformInfo() {
  using namespace std;
  auto st = [&](vector<int> vc) {
    int ret = 0;
    for (int val : vc)
      ret += 1 << val;
    return ret;
  };
  const vector<int> needTimes = {-1, 50, 50, 50, 500, 500, 500, 1000, 1, 1};
  const vector<int> needBuys = {-1,         0,
                                0,          0,
                                st({1, 2}), st({1, 3}),
                                st({2, 3}), st({4, 5, 6}),
                                st({7}),    st({1, 2, 3, 4, 5, 6, 7})};
  const vector<int> sells = {-1, 1, 2, 3, 4, 5, 6, 7, 0, 0};

  mapToPlatID.assign(MAP_SIZE, std::vector<int>(MAP_SIZE, -1));
  for (int i = 0; i < MAP_SIZE; ++i) {
    string &s = g[i];
    for (int j = 0; j < MAP_SIZE; ++j) {
      double x = j * 0.5 + 0.25;
      double y = (MAP_SIZE - 1 - i) * 0.5 + 0.25;
      if (!role) {
        if (s[j] == 'A') {
          int id = robots.size();
          robots.emplace_back(Robot(id, x, y));
        } else if (s[j] >= '1' && s[j] <= '9') {
          int id = plats.size();
          mapToPlatID[i][j] = id;
          int type = s[j] - '0';
          plats.emplace_back(Platform(id, type, needTimes[type], needBuys[type],
                                      sells[type], 1, x, y));
          bool ok1 = true, ok2 = true;
          for (int ii = i - 1; ii <= i + 1; ++ii)
            for (int jj = j - 3; jj <= j + 3; ++jj)
              if (ii < 0 || ii >= MAP_SIZE || jj < 0 || jj >= MAP_SIZE ||
                  g[ii][jj] == '#')
                ok1 = false;
          for (int ii = i - 3; ii <= i + 3; ++ii)
            for (int jj = j - 1; jj <= j + 1; ++jj)
              if (ii < 0 || ii >= MAP_SIZE || jj < 0 || jj >= MAP_SIZE ||
                  g[ii][jj] == '#')
                ok2 = false;
          plats.back().rushAble = ok1 | ok2;
        }
      } else {
        if (s[j] == 'B') {
          int id = robots.size();
          robots.emplace_back(Robot(id, x, y));
        } else if (s[j] >= 'a' && s[j] <= 'i') {
          int id = plats.size();
          mapToPlatID[i][j] = id;
          int type = s[j] - 'a' + 1;
          plats.emplace_back(Platform(id, type, needTimes[type], needBuys[type],
                                      sells[type], 1, x, y));
          bool ok1 = true, ok2 = true;
          for (int ii = i - 1; ii <= i + 1; ++ii)
            for (int jj = j - 3; jj <= j + 3; ++jj) {
              if ((ii == i - 1 || ii == j + 1) && (jj == j - 3 || jj == j + 3))
                continue;
              if (ii < 0 || ii >= MAP_SIZE || jj < 0 || jj >= MAP_SIZE ||
                  g[ii][jj] == '#')
                ok1 = false;
            }
          for (int ii = i - 3; ii <= i + 3; ++ii)
            for (int jj = j - 1; jj <= j + 1; ++jj) {
              if ((ii == i - 3 || ii == j + 3) && (jj == j - 1 || jj == j + 1))
                continue;
              if (ii < 0 || ii >= MAP_SIZE || jj < 0 || jj >= MAP_SIZE ||
                  g[ii][jj] == '#')
                ok2 = false;
            }
          plats.back().rushAble = ok1 | ok2;
        }
      }
    }
  }
}

LL Map::getHash() {
  const int M1 = 998244353, M2 = 1e9 + 9, B1 = 1331, B2 = 131;
  LL h1 = 0, h2 = 0;
  for (int i = 0; i < MAP_SIZE; i++) {
    for (int j = 0; j < MAP_SIZE; j++) {
      h1 = (h1 * B1 + g[i][j]) % M1;
      h2 = (h2 * B2 + g[i][j]) % M2;
    }
  }
  return h1 << 31 | h2;
}

void Map::getMapId() {
  const std::vector<LL> MAP_ID = {
      -1,
  };

  LL hash = getHash();
  dbg("hash code:", hash);

  mapId = 0;
  for (int i = 1; i < MAP_ID.size(); i++) {
    if (hash == MAP_ID[i]) {
      mapId = i;
      break;
    }
  }

  dbg("map id:", mapId);
}

void Map::getMapType() {
  std::vector<int> platNumForBlue(10);
  for (int i = 0; i < MAP_SIZE; ++i)
    for (int j = 0; j < MAP_SIZE; ++j)
      if (g[i][j] >= '1' && g[i][j] <= '9')
        ++platNumForBlue[g[i][j] - '0'];
  int sum = 0;
  for (int i = 4; i <= 7; ++i)
    sum += platNumForBlue[i];
  sum += platNumForBlue[9];
  if (sum == 0) {
    // 123类型工作台和8类型工作台随意组合无所谓
    type = 4;
    return;
  }

  // dbg("not type 4");

  int n = MAP_SIZE;
  vector<vector<int>> gg(n, vector<int>(n));
  auto isBlock = [&](int i, int j) {
    return i < 0 || i >= n || j <= 0 || j >= n || g[i][j] == '#';
  };
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j)
      if (g[i][j] == '#')
        gg[i][j] = -1;
      else {
        if (isBlock(i - 1, j) && isBlock(i + 1, j) ||
            isBlock(i, j - 1) && isBlock(i, j + 1))
          gg[i][j] = -1;
      }

  auto &logger = Logger::instance();
  logger.load("log_xxx.txt");
  for (int i = 0; i < n; ++i) {
    string line;
    for (int j = 0; j < n; ++j)
      if (gg[i][j] == -1)
        line += '#';
      else
        line += '.';
    logger.write(line);
  }

  vector<vector<int>> to(n * n);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j)
      if (gg[i][j] != -1) {
        for (int dx = -1; dx <= 1; ++dx)
          for (int dy = -1; dy <= 1; ++dy)
            if (dx || dy) {
              if (dx && dy)
                continue;
              int ii = i + dx;
              int jj = j + dy;
              if (ii < 0 || ii >= n || jj < 0 || jj >= n || gg[ii][jj] == -1)
                continue;
              to[i * n + j].push_back(ii * n + jj);
            }
      }

  std::vector<int> leader(n * n);
  std::vector<int> color(n * n);
  std::function<int(int)> getLeader = [&](int x) {
    return leader[x] == x ? x : leader[x] = getLeader(leader[x]);
  };
  iota(leader.begin(), leader.end(), 0);
  for (int u = 0; u < n * n; ++u) {
    for (int v : to[u]) {
      logger.write(u, v);
      leader[getLeader(u)] = getLeader(v);
    }
  }
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) {
      int u = i * n + j;
      if (g[i][j] >= '1' && g[i][j] <= '9')
        color[getLeader(u)] |= 1;
      else if (g[i][j] >= 'a' && g[i][j] <= 'i')
        color[getLeader(u)] |= 2;
    }
  if ((*std::max_element(color.begin(), color.end())) < 3) {
    type = 2;
    return;
  }

  // dbg("not type 2");

  auto cal = [&](std::string &s) {
    int sz = s.size();
    int ans = 0;
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j + sz <= n; ++j) {
        int u = i * n + j + sz / 2;
        if (!color[getLeader(u)])
          continue;
        bool ok = true;
        for (int k = 0; k < sz; ++k)
          if (g[i][j + k] != s[k]) {
            ok = false;
            break;
          }
        ans += ok;
      }
    }
    for (int i = 0; i + sz <= n; ++i) {
      for (int j = 0; j < n; ++j) {
        int u = (i + sz / 2) * n + j;
        if (!color[getLeader(u)])
          continue;
        bool ok = true;
        for (int k = 0; k < sz; ++k)
          if (g[i + k][j] != s[k]) {
            ok = false;
            break;
          }
        ans += ok;
      }
    }
    return ans;
  };
  std::string s;
  s = "#..#";
  sum = 0;
  sum += cal(s);
  s = "#...#";
  sum += cal(s);
  s = "#....#";
  sum += cal(s);

  dbg("narrow SUM: ", sum);
  int T = 250; // 参数，地图类型判定阈值
  if (sum > T) {
    type = 3;
    return;
  }

  // dbg("not type3");

  type = 1;
}

void Map::initNodeCosts() {
  for (auto &[p, cost] : nodes) {
    std::vector<Pt> os = Obstacle::getNearbyObstaclePoints(g, p, 3);
    for (auto &o : os) {
      int kx = (p.x > o.x + 0.5) - (p.x < o.x);
      int ky = (p.y > o.y + 0.5) - (p.y < o.y);
      if (!kx && !ky) {
        cost = 1e9;
        break;
      }

      D d;
      if (kx < 0) {
        d = ky ? std::sqrt(sqr(o.x - p.x) +
                           sqr(ky < 0 ? o.y - p.y : o.y + 0.5 - p.y))
               : o.x - p.x;
      } else if (!kx) {
        d = ky < 0 ? o.y - p.y : p.y - o.y - 0.5;
      } else {
        d = ky ? std::sqrt(sqr(o.x + 0.5 - p.x) +
                           sqr(ky < 0 ? o.y - p.y : o.y + 0.5 - p.y))
               : p.x - o.x - 0.5;
      }
      myAssert(d > 0);
      if (d <= 2) {
        smax(cost, 1.5 / d);
      }
    }
  }
}

void Map::init(int _role, std::vector<std::string> &_g, bool _isExtendPlats,
               int graphD, const std::vector<Pt> &basicSamplPts) {
  role = _role;
  g = _g;
  isExtendPlats = _isExtendPlats;

  getRobotAndPlatformInfo();
  getMapId();
  initNodes(basicSamplPts);
  initNodeCosts();
  dbg("initNodes OK");
  buildGraph(graphD);
  dbg("buildGraph OK");
  initEvalDisPP();
  dbg("initEvalDisPP OK");
  getMapType();
  dbg("mapType: ", type);
}

D Map::getEvalDis(Pt p, int pid, int area, D r) {
  if (pid < 0)
    return 1e9;

  auto &dis = getDistTableFromPlat(pid, area, r);
  D ans = 1e9;
  for (int i : getReachableNeighbors(p, r, GET_NEXT_TO_NEIGHBOR_RANGE)) {
    smin(ans, dis[i].distEx + nodes[i].p.dis(p));
  }
  return ans;
}

void Map::initEvalDisPP() {
  evalDisPP.resize(plats.size() * 5);
  for (auto &p1 : plats) {
    for (int k : p1.validAreas) {
      auto &e = evalDisPP[p1.id * 5 + k];
      e.resize(plats.size(), 1e9);
      auto &dt = getDistTableFromPlat(p1.id, k, R[1]);
      for (auto &p2 : plats) {
        for (int i : getPlatformNeighbors(p2.id)) {
          smin(e[p2.id], dt[i].distEx);
        }
      }
    }
  }
}

void Map::initNodes(const std::vector<Pt> &basicSamplPts) {
  mapToNodeID.assign(GN, std::vector<std::vector<int>>(GN));
  // for (int i = 0; i < GN; i++) {
  //   for (int j = 0; j < GN; j++) {
  //     addNode(Pt(i * GSTEP, j * GSTEP));
  //   }
  // }

  for (auto &p : basicSamplPts) {
    addNode(p);
  }
  for (auto &p : plats) {
    addNode(p.p);
    for (int i = 1; i <= 5; i++) {
      addNode(Pt(p.p.x + 0.25 * i, p.p.y));
      addNode(Pt(p.p.x - 0.25 * i, p.p.y));
      addNode(Pt(p.p.x, p.p.y + 0.25 * i));
      addNode(Pt(p.p.x, p.p.y - 0.25 * i));
    }
  }

  platIsKey.resize(plats.size());
  if (!isExtendPlats)
    return;

  { // init Key Platforms

    auto check = [&](std::vector<std::string> str, int val) -> void {
      int sn = str.size(), sm = str[0].size();
      for (int i = 0; i + sn <= MAP_SIZE; i++) {
        for (int j = 0; j + sm <= MAP_SIZE; j++) {
          bool flag = true;
          for (int x = 0; x < sn; x++) {
            for (int y = 0; y < sm; y++) {
              if (str[x][y] == '#' && g[i + x][j + y] != '#' ||
                  str[x][y] != '#' && g[i + x][j + y] == '#') {
                flag = false;
                goto FlagKey;
              }
            }
          }
        FlagKey:
          if (flag) {
            for (int x = 0; x < sn; x++) {
              for (int y = 0; y < sm; y++) {
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
  for (int i = 0; i < 8; i++) {
    dirs.push_back(PI / 4 * i);
  }

  auto get = [&](D x) { return x / 180.0 * PI; };

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

  auto add = [&](Pt p) {
    for (Pt dp : dps) {
      addNode(p + dp * 0.4);
      addNode(p + dp * 0.35);
      addNode(p + dp * 0.3);
      addNode(p + dp * 0.2);
      addNode(p + dp * 0.15);
      addNode(p + dp * 0.10);
      addNode(p + dp * 0.05);
    }
  };

  for (auto &plat : plats) {
    add(plat.p);
    if (platIsKey[plat.id]) {
      for (Pt dp : dps) {
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

int Map::addNode(Pt pos) {
  if (pos.x < 0 || pos.y < 0 || pos.x >= 50.0 || pos.y >= 50.0)
    return -1;
  auto it = posToNodeID.find(pos);
  if (it == posToNodeID.end()) {
    it = posToNodeID.emplace(pos, nodes.size()).first;
    mapToNodeID[pos.x / GSTEP][pos.y / GSTEP].push_back(it->second);
    nodes.emplace_back(pos);
  }
  return it->second;
}

std::vector<int> Map::getReachableNeighbors(Pt p, D radius, int d) {
  std::vector<Pt> os = Obstacle::getNearbyObstaclePoints(g, p, d / 2 + 3);

  std::vector<int> neighbors;
  int x = p.x / GSTEP, y = p.y / GSTEP;
  for (int i = std::max(0, x - d); i <= x + d && i < GN; i++) {
    for (int j = std::max(0, y - d); j <= y + d && j < GN; j++) {
      for (auto id : mapToNodeID[i][j]) {
        const Pt &q = nodes[id].p;

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

std::vector<int> Map::getPlatformNeighbors(int pid) {
  std::vector<int> s;
  auto &p = plats[pid];
  int x = p.p.x / GSTEP, y = p.p.y / GSTEP;
  int d = 2;
  for (int i = std::max(0, x - d); i <= x + d && i < GN; i++) {
    for (int j = std::max(0, y - d); j <= y + d && j < GN; j++) {
      for (int id : mapToNodeID[i][j]) {
        if (nodes[id].p.dis(p.p) <= 0.4) {
          s.push_back(id);
        }
      }
    }
  }
  return s;
}

std::vector<int> Map::getPlatformNeighbors2(int pid) {
  D rad = platIsKey[pid] ? 0.7 : 0.4;
  std::vector<int> s;
  auto &p = plats[pid];
  int x = p.p.x / GSTEP, y = p.p.y / GSTEP;
  int d = 2;
  for (int i = std::max(0, x - d); i <= x + d && i < GN; i++) {
    for (int j = std::max(0, y - d); j <= y + d && j < GN; j++) {
      for (int id : mapToNodeID[i][j]) {
        if (nodes[id].p.dis(p.p) <= rad) {
          s.push_back(id);
        }
      }
    }
  }
  return s;
}

bool Map::queryReachability(Pt a, Pt b, D radius) {
  if (a == b)
    return true;

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
            g[i][j] == '#') {
          D u = j * 0.5, v = (MAP_SIZE - 1 - i) * 0.5;
          if (Obstacle::checkOS(Pt(u, v), a, b, radius))
            return false;
        }
      }
    }
    return true;
  }

  // TODO
  // if (rx - lx > ry - ly) {
  //   auto f = [&](int x) -> int {
  //     D y = (MAP_SIZE - 1 - x) * 0.5 + 0.25;
  //     D t = (a.x - b.x) / (a.y - b.y) * (y - a.y) + a.x;
  //     return std::floor(t * 2);
  //   };

  //   for (int i = lx; i <= rx; i++) {
  //     int y = f(i);
  //     for (int j = std::max(-1, y - 3); j <= y + 3 && j <= MAP_SIZE; j++) {
  //       if (i < 0 || j < 0 || i == MAP_SIZE || j == MAP_SIZE ||
  //           g[i][j] == '#') {
  //         D u = j * 0.5, v = (MAP_SIZE - 1 - i) * 0.5;
  //         if (Obstacle::checkOS(Pt(u, v), a, b, radius))
  //           return false;
  //       }
  //     }
  //   }
  // } else {
  //   auto f = [&](int y) -> int {
  //     D x = 0.5 * y + 0.25;
  //     D t = (a.y - b.y) / (a.x - b.x) * (x - a.x) + a.y;
  //     return std::floor(MAP_SIZE - t * 2);
  //   };
  //   for (int j = ly; j <= ry; j++) {
  //     int x = f(j);
  //     for (int i = std::max(-1, x - 3); i <= x + 3 && i <= MAP_SIZE; i++) {
  //       if (i < 0 || j < 0 || i == MAP_SIZE || j == MAP_SIZE ||
  //           g[i][j] == '#') {
  //         D u = j * 0.5, v = (MAP_SIZE - 1 - i) * 0.5;
  //         if (Obstacle::checkOS(Pt(u, v), a, b, radius))
  //           return false;
  //       }
  //     }
  //   }
  // }
  return true;
}

Pt Map::makeWayTo(Pt cur, D r, std::vector<std::pair<Pt, Pt>> &segs, D dist) {
  int t = -1;
  auto ns = getReachableNeighbors(cur, r, 5);
  for (auto o : ns) {
    Pt u = cur, v = nodes[o].p;
    if (!segs.empty()) {
      D dis1 = 1e9, dis2 = 1e9;
      D f = 1.5, g = 2.5;
      for (auto &[a, b] : segs) {
        if (distSS(a, b, u, v) < f) {
          smin(dis1, distPS(v, a, b));
          smin(dis2, distPS(u, a, b));
        }
      }
      if (dis2 <= g) {
        D z = (dis2 - dis1) * 1e6;
        if (smin(dist, z)) {
          t = o;
        }
      }
    }
  }
  return t >= 0 ? nodes[t].p : cur;
}

std::vector<int> Map::getPath(Pt cur, int pid, int area, D r) {
  if (pid < 0)
    return {};
  auto &d = getDistTableFromPlat(pid, area, r);

  D dist = 1e9;
  int t = -1;

  auto ns = getReachableNeighbors(cur, r, GET_NEXT_TO_NEIGHBOR_RANGE);

  for (auto o : ns) {
    if (smin(dist, d[o].distEx + nodes[o].p.dis(cur))) {
      t = o;
    }
  }

  if (t == -1)
    return {};

  std::vector<int> path;
  while (t != -1) {
    path.push_back(t);
    t = d[t].par;
  }
  return path;
}

Pt Map::getNextTo(Pt cur, int pid, int area, D r) {
  auto path = getPath(cur, pid, area, r);
  if (path.empty())
    return cur;
  int i = 0;
  if (cur.dis(nodes[path[i]].p) < 0.1)
    i++;
  for (i++; i < path.size(); i++) {
    if (!queryReachability(cur, nodes[path[i]].p, r + 0.1)) {
      return nodes[path[i - 1]].p;
    }
  }

  return nodes[path.back()].p;
}
Pt Map::getNextTo(Robot &r) {
  int pid = r.getTarget(), area = r.state == 1 ? r.buyPlatArea : 0;
  auto path = getPath(r.p, pid, area, r.radius);
  // r.path = path;
  if (path.empty())
    return r.p;

  r.path = path;
  r.pathType = "dijkstra";

  int i = 0;
  if (r.p.dis(nodes[path[i]].p) < 0.1)
    i++;
  for (i++; i < path.size(); i++) {
    if (!queryReachability(r.p, nodes[path[i]].p, r.radius + 0.1)) {
      return nodes[path[i - 1]].p;
    }
  }

  return nodes[path.back()].p;
}

void Map::buildGraph(int d) {
  for (int k = 0; k < 2; k++) {
    graph[k] = new Graph(nodes);
    auto &g = *graph[k];

    for (int i = 0; i < nodes.size(); i++) {
      auto ns = getReachableNeighbors(nodes[i].p, R[k], d);
      for (int j : ns) {
        g.addEdge(j, i, (nodes[i].p - nodes[j].p).len());
      }
    }

    g.buildEdges();

    dbg(g.n, g.m);

    distTableFromPlat[k].resize(plats.size() * 5);

    for (auto &p : plats) {
      auto tt = getPlatformNeighbors2(p.id);

      DistTable ns;
      for (int o : tt) {
        D cost = nodes[o].p.dis(p.p) <= 0.4 ? 0.0 : 1e9;
        ns.emplace_back(cost, cost, o);
      }
      getDistTableFromPlat(p.id, 0, R[k]) = g.dijkstra(ns, role);

      if (platIsKey[p.id]) {
        if (platIsKey[p.id] == 1) {
          p.validAreas = {3, 4};
        } else {
          p.validAreas = {1, 2};
        }
        for (int area : p.validAreas) {
          DistTable newNs;
          for (auto o : ns) {
            if (checkPointArea(area, nodes[o.par].p - p.p)) {
              if (o.dist == 1e9) {
                o.dist = o.distEx = 5;
              }
              newNs.push_back(o);
            }
          }
          getDistTableFromPlat(p.id, area, R[k]) = g.dijkstra(newNs, role);
        }
      } else {
        p.validAreas = {0};
      }
    }

    for (auto c : customStartPoint) {
      getDistTableFromCustom(c, R[k]) = g.dijkstra({DistInfo(0, 0, c)}, role);
    }
  }
}
