#pragma once
#include "Geometry.hpp"
#include "Platform.hpp"
#include "Robot.hpp"
#include "Tools.hpp"
#include "constants.hpp"

class NewMap {
private:
  NewMap() {}
  void getRobotAndPlatformInfo();

  std::set<Pt> pset;

public:
  NewMap(const NewMap &) = delete;
  NewMap &operator=(const NewMap &) = delete;
  NewMap(NewMap &&) = delete;
  NewMap &operator=(NewMap &&) = delete;

  static auto &instance() {
    static NewMap t;
    return t;
  }

  int map_id = 0;
  int n;
  std::vector<std::string> origin_map, g, h;
  std::vector<Robot> robots;
  std::vector<Platform> plats;

  long long geths() {
    const int mod1 = 998244353, mod2 = 1e9 + 7;
    const int base1 = 2333, base2 = 3931;
    int hs1 = 0, hs2 = 0;
    for (auto &row : h)
      for (char c : row) {
        hs1 = (1ll * hs1 * base1 + c) % mod1;
        hs2 = (1ll * hs2 * base2 + c) % mod2;
      }
    return 1ll * hs1 * 2000000000 + hs2;
  }

  std::vector<Pt> ps;
  std::vector<std::vector<std::vector<int>>> ids;
  std::vector<std::vector<int>> mapToPlatID;

  int gn;
  D gstep;

  void addVertex(Pt pos) {
    if (pos.x < 0 || pos.y < 0 || pos.x >= 50.0 || pos.y >= 50.0)
      return;
    if (!pset.insert(pos).second)
      return;
    ids[pos.x / gstep][pos.y / gstep].push_back(ps.size());
    ps.push_back(pos);
  }

  using DistInfo = std::vector<std::pair<D, int>>;
  std::vector<std::vector<DistInfo>> distFromPlat[2];

  int getPlatAreaNum(int pid) { return distFromPlat[0][pid].size(); }

  void addPlatformExtraVertex() {

    for (auto &plat : plats) {
      // if (Obstacle::getNearbyObstaclePoints(origin_map, plat.p, 1).size() <
      // 1)
      //   continue;
      std::vector<D> dirs;
      for (int i = 0; i < 8; i++) {
        dirs.push_back(PI / 4 * i);
      }

      auto get = [&](D x) {
        return x / 180.0 * PI;
      };

      if (isKey[plat.id]) {
        D t = get(29.5);
        dirs.push_back(-PI / 2 + t);
        dirs.push_back(-PI / 2 - t);
        dirs.push_back(PI / 2 + t);
        dirs.push_back(PI / 2 - t);
        dirs.push_back(t);
        dirs.push_back(-t);
        dirs.push_back(PI + t);
        dirs.push_back(PI - t);
      }
      
      for (D d : dirs) {
        Pt dp(std::cos(d), std::sin(d));
        addVertex(plat.p + dp * 0.4);
        addVertex(plat.p + dp * 0.35);
        addVertex(plat.p + dp * 0.3);
        addVertex(plat.p + dp * 0.2);
        addVertex(plat.p + dp * 0.15);
        addVertex(plat.p + dp * 0.10);
        addVertex(plat.p + dp * 0.05);

        if (isKey[plat.id]) {
          // addVertex(plat.p + dp * 0.45);
          addVertex(plat.p + dp * 0.65);
        }
      }
    }
  }

  std::vector<int> getPlatformNeighbors(int pid) {
    std::vector<int> s;
    auto &p = plats[pid];
    int x = p.p.x / gstep, y = p.p.y / gstep;
    int d = 2;
    for (int i = std::max(0, x - d); i <= x + d && i < gn; i++) {
      for (int j = std::max(0, y - d); j <= y + d && j < gn; j++) {
        for (int id : ids[i][j]) {
          if (ps[id].dis(p.p) <= 0.4) {
            s.push_back(id);
          }
        }
      }
    }
    return s;
  }
  
  std::vector<int> getPlatformNeighbors2(int pid) {
    D rad = isKey[pid] ? 0.7 : 0.4;
    std::vector<int> s;
    auto &p = plats[pid];
    int x = p.p.x / gstep, y = p.p.y / gstep;
    int d = 2;
    for (int i = std::max(0, x - d); i <= x + d && i < gn; i++) {
      for (int j = std::max(0, y - d); j <= y + d && j < gn; j++) {
        for (int id : ids[i][j]) {
          if (ps[id].dis(p.p) <= rad) {
            s.push_back(id);
          }
        }
      }
    }
    return s;
  }

  void init(int n_ = MAP_SIZE) {
    n = n_;
    g.resize(n);
    for (int i = 0; i < n; i++)
      std::cin >> g[i];
    origin_map = g;
    getRobotAndPlatformInfo();
    h.resize(n);
    for (int j = 0; j < n; ++j)        // 升序枚举x坐标
      for (int i = n - 1; i >= 0; --i) // 升序枚举y坐标
        h[j] += g[i][j];
    gn = n * DIVIDE_SIZE;
    ids.assign(gn, std::vector<std::vector<int>>(gn));
    gstep = 50.0 / gn;
    for (int i = 0; i < gn; i++) {
      for (int j = 0; j < gn; j++) {
        addVertex(Pt(i * gstep, j * gstep));
        // ps.emplace_back(i * gstep, j * gstep);
      }
    }

    initKey();
    addPlatformExtraVertex();

    buildGraph();

    init_can_to();
    initEvalDisPP();
  }

  std::vector<int> getReachableNeighbors(Pt p, D radius, int d);

  void buildGraph();

  // 查询 a->b 半径为 radius 是否可达
  bool queryReachability(Pt a, Pt b, D radius);

  std::vector<Segment> aroundObs(Point p, int d = 3) {
    // return {};
    int i = p.x / 0.25;
    int j = p.y / 0.25;
    std::vector<Segment> ans;
    for (int ii = i - d; ii <= i + d; ++ii) {
      std::string info;
      for (int jj = j - d; jj <= j + d; ++jj) {
        if (ii >= 0 && ii < gn && jj >= 0 && jj < gn &&
            h[ii / 2][jj / 2] == '#' && ii % 2 == 1 && jj % 2 == 1) {
          ans.push_back({Point((ii - 1) * 0.25, (jj - 1) * 0.25),
                         Point((ii - 1) * 0.25, (jj + 1) * 0.25)});
          ans.push_back({Point((ii - 1) * 0.25, (jj - 1) * 0.25),
                         Point((ii + 1) * 0.25, (jj - 1) * 0.25)});
          ans.push_back({Point((ii + 1) * 0.25, (jj + 1) * 0.25),
                         Point((ii - 1) * 0.25, (jj + 1) * 0.25)});
          ans.push_back({Point((ii + 1) * 0.25, (jj + 1) * 0.25),
                         Point((ii + 1) * 0.25, (jj - 1) * 0.25)});
        }
      }
    }
    return ans;
  }

  std::vector<std::vector<LL>> canTo[2];

  void init_can_to() {
    assert(plats.size() <= 63);
    for (int k = 0; k < 2; k++) {
      canTo[k].assign(gn, std::vector<LL>(gn));
      for (int x = 0; x < gn; x++) {
        for (int y = 0; y < gn; y++) {
          for (int pid = 0; pid < plats.size(); pid++) {
            int d = 2;
            for (int i = std::max(0, x - d); i <= x + d && i < gn; i++) {
              for (int j = std::max(0, y - d); j <= y + d && j < gn; j++) {
                for (int id : ids[i][j]) {
                  if (distFromPlat[k][pid][0][id].first < 1e9) {
                    canTo[k][x][y] |= 1LL << pid;
                    goto ee;
                  }
                }
              }
            }
          ee:;
          }
        }
      }
    }
  }

  bool can_to(Pt cur, int pid, D r) {
    if (pid < 0)
      return false;
    int x = cur.x / gstep, y = cur.y / gstep, k = !cmp(r, R[1]);
    return canTo[k][x][y] >> pid & 1;

    /* auto &dis = distFromPlat[!cmp(r, R[1])][pid];
     // auto ns = getReachableNeighbors(cur, r);
     // std::cerr << ns.size() << "\n";
     // for (int i : ns) {
     //   if (dis[i].first < 1e9)
     //     return true;
     // }

     myAssert(gstep == 0.25);
     int x = cur.x / gstep;
     int y = cur.y / gstep;

     int d = 1;
     for (int i = std::max(0, x - d); i <= x + d && i < gn; i++) {
       for (int j = std::max(0, y - d); j <= y + d && j < gn; j++) {
         if (dis[i * gn + j].first < 1e9)
           return true;
       }
     }

     return false;*/
  }

  D getEvalDis(Pt p, int pid, int area, D r) {
    if (pid < 0)
      return 1e9;

    auto &dis = distFromPlat[!cmp(r, 0.53)][pid][area];
    D ans = 1e9;
    for (int i : getReachableNeighbors(p, r, 2)) {
      smin(ans, dis[i].first + ps[i].dis(p));
    }
    return ans;
  }

  std::vector<std::vector<std::vector<D>>> evalDisPP;
  void initEvalDisPP() {
    evalDisPP.resize(plats.size());
    for (auto &p1 : plats) {
      auto &e = evalDisPP[p1.id];
      e.resize(getPlatAreaNum(p1.id));
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

  Pt getNextTo(Pt, int, int, D, std::vector<Segment> &);

  std::vector<int> isKey;
  void initKey() {
    isKey.resize(plats.size());
    {

      auto check = [&](std::vector<std::string> str, int val) -> void {
        int sn = str.size(), sm = str[0].size();
        for (int i = 0; i + sn <= n; i++) {
          for (int j = 0; j + sm <= n; j++) {
            bool flag = true;
            for (int x = 0; x < sn; x++) {
              for (int y = 0; y < sm; y++) {
                if (str[x][y] == '#' && origin_map[i + x][j + y] != '#' ||
                    str[x][y] != '#' && origin_map[i + x][j + y] == '#') {
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
                    isKey[id] = val;
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
  }
};