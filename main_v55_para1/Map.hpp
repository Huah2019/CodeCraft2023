#pragma once

#include "Foe.hpp"
#include "Graph.hpp"
#include "Obstacle.hpp"
#include "Platform.hpp"
#include "Robot.hpp"
#include "utils.hpp"

using namespace std;

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
  static auto &instanceFoe() {
    static Map t;
    return t;
  }

  int mapId;
  int role; // role=0表示Blue,role=1表示Red
  int type; // 地图类型
  Graph *graph[2];
  bool isExtendPlats;

  std::vector<std::string> g;
  std::vector<Foe> foes, aliveFoes;

  std::vector<Robot> robots;
  std::vector<Platform> plats;
  std::vector<std::vector<int>> mapToPlatID;
  std::vector<int> platIsKey;

  std::vector<Node> nodes;
  std::vector<std::vector<std::vector<int>>> mapToNodeID;
  std::map<Pt, int> posToNodeID;
  // std::vector<std::vector<DistInfo>> distFromPlat[2];
  std::vector<DistTable> distTableFromPlat[2];

  DistTable &getDistTableFromPlat(int pid, int area, D radius) {
    return distTableFromPlat[!cmp(radius, R[1])][pid * 5 + area];
  }

  std::vector<int> customStartPoint;
  std::vector<DistTable> distTableFromCustom[2];
  DistTable &getDistTableFromCustom(int cid, D radius) {
    return distTableFromCustom[!cmp(radius, R[1])][cid];
  }

  int addCustomStartPoint(Pt p) {
    int id = addNode(p);
    customStartPoint.push_back(id);
    return customStartPoint.size() - 1;
  }

  void getRobotAndPlatformInfo();
  LL getHash();
  void getMapId();
  void getMapType();
  void init(int, std::vector<std::string> &, bool, int, const std::vector<Pt> &);

  int addNode(Pt pos);
  void initNodes(const std::vector<Pt> &basicSamplPts);

  std::vector<int> getReachableNeighbors(Pt p, D radius, int d);

  void initNodeCosts();
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

  D getEvalDis(Pt p, int pid, int area, D r);

  std::vector<std::vector<D>> evalDisPP;
  void initEvalDisPP();
  D getEvalDisPP(int p1, int area, int p2) { return evalDisPP[p1 * 5 + area][p2]; }

  D getEvalDis(Robot &rob) {
    return getEvalDis(rob.p, rob.getTarget(), 0, rob.radius);
  }
};
