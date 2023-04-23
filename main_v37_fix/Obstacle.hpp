#pragma once
#include "constants.hpp"
#include "geo.hpp"

using namespace std;

struct Obstacle {
  Pt p;
  int k;
  Pt pt() {
    return Pt{p.x + (k == 1 || k == 2 ? 0.5 : 0), p.y + (k >= 2 ? 0.5 : 0)};
  }
  Obstacle prev() {
    auto o = *this;
    o.k = (o.k + 3) & 3;
    return o;
  }
  Obstacle next() {
    auto o = *this;
    o.k = (o.k + 1) & 3;
    return o;
  }
  Pt unitDir() { return (next().pt() - pt()).unit(); }

  template <class T>
  static std::vector<Pt> getNearbyObstaclePoints(T &map, Pt p, int d) {
    std::vector<Pt> os;

    int x = MAP_SIZE - p.y * 2, y = p.x * 2;
    for (int i = std::max(-1, x - d); i <= MAP_SIZE && i <= x + d; i++) {
      for (int j = std::max(-1, y - d); j <= MAP_SIZE && j <= y + d; j++) {
        if (i < 0 || j < 0 || i == MAP_SIZE || j == MAP_SIZE ||
            map[i][j] == '#') {
          D u = j * 0.5, v = (MAP_SIZE - 1 - i) * 0.5;
          os.emplace_back(u, v);
        }
      }
    }

    return os;
  }

  template <class T>
  static std::vector<Obstacle> getNearbyObstacles(T &map, Pt p, int d) {

    std::vector<Obstacle> os;
    for (auto v : getNearbyObstaclePoints(map, p, d)) {
      for (int i = 0; i < 4; i++) {
        os.push_back({v, i});
      }
    }

    return os;
  }

  // 判断线段 (a, b) 是否与障碍的距离不超过 radius
  static bool checkOS(const Pt &o0, const Pt &a, const Pt &b, D radius) {
    // radius -= 0.015;
    // Pt o1(o0.x + 0.5, o0.y), o2(o1.x, o1.y + 0.5), o3(o0.x, o2.y);

    // myAssert(!(a == b));
    // return 
    // // checkSS(o0, o1, a, b) || checkSS(o1, o2, a, b) ||
    // //        checkSS(o2, o3, a, b) || checkSS(o3, o0, a, b) ||
    //        distPS(o0, a, b) <= radius ||
    //        distPS(o1, a, b) <= radius ||
    //        distPS(o2, a, b) <= radius ||
    //        distPS(o3, a, b) <= radius;
    radius -= 0.02;
    Pt o1(o0.x + 0.5, o0.y), o2(o1.x, o1.y + 0.5), o3(o0.x, o2.y);

    // if (a == b)
    // return (a - p).len();
    // D d = (b - a).len2(), t = std::min(d, std::max(0.0, dot(b - a, p - a)));
    // return ((p - a) * d - (b - a) * t).len() / d;
  
    Pt v = b - a;
    D d = v.len2();
    radius *= d;
    radius *= radius;
    #define F(p) ((p - a) * d - v * std::min(d, std::max(0.0, dot(v, p - a)))).len2()
    return F(o0) <= radius || F(o1) <= radius || F(o2) <= radius || F(o3) <= radius;
    #undef F
  }
};
