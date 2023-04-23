#pragma once

#include "Foe.hpp"
#include "Graph.hpp"
#include "Obstacle.hpp"
#include "Platform.hpp"
#include "Robot.hpp"
#include "utils.hpp"

using namespace std;

using DistInfo = std::vector<std::pair<D, int>>;

struct Graph;

class Map {
private:
  Map() {}
  std::set<Pt> s;

public:
  Map(const Map &) = delete;
  Map &operator=(const Map &) = delete;
  Map(Map &&) = delete;
  Map &operator=(Map &&) = delete;

  static auto &instance() {
    static Map t;
    return t;
  }
  static auto &instanceFoe() {
    static Map tFoe;
    return tFoe;
  }

  int mapId;
  int role; // role=0表示Blue,role=1表示Red
  int type; // 地图类型
  Graph *graph[2];
  bool isExtendPlats;

  std::vector<std::string> g;
  std::vector<Foe> foes;

  std::vector<Robot> robots;
  std::vector<Platform> plats;
  std::vector<std::vector<int>> mapToPlatID;
  std::vector<int> platIsKey;

  std::vector<Node> nodes;
  std::vector<std::vector<std::vector<int>>> mapToNodeID;

  std::vector<std::vector<DistInfo>> distFromPlat[2];

  void getRobotAndPlatformInfo();
  LL getHash();
  void getMapId();
  void getMapType();
  void init(int, std::vector<std::string> &, bool, int, const std::vector<Pt> &);

  void addNode(Pt pos);
  void initNodes(const std::vector<Pt> &basicSamplPts);

  std::vector<int> getReachableNeighbors(Pt p, D radius, int d);

  void buildGraph(int);

  std::vector<int> getPlatformNeighbors(int pid);

  std::vector<int> getPlatformNeighbors2(int pid);

  // 查询 a->b 半径为 radius 是否可达
  bool queryReachability(Pt a, Pt b, D radius);

  Pt makeWayTo(Pt cur, D r, std::vector<std::pair<Pt, Pt>> &, D);
  std::vector<int> getPath(Pt cur, int pid, int area, D r);
  Pt getNextTo(Pt cur, int pid, int area, D r);
  Pt getNextTo(Robot &r);
  // void findNextTo(Robot &r);

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

  D getEvalDis(Robot &rob) {
    return getEvalDis(rob.p, rob.getTarget(), 0, rob.radius);
  }
};
