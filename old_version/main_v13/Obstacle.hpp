#pragma once
#include "constants.hpp"
#include "geo.hpp"

using std::abs;

struct Obstacle
{
  Pt p;
  int k;
  Pt pt()
  {
    return Pt{p.x + (k == 1 || k == 2 ? 0.5 : 0), p.y + (k >= 2 ? 0.5 : 0)};
  }
  Obstacle prev()
  {
    auto o = *this;
    o.k = (o.k + 3) & 3;
    return o;
  }
  Obstacle next()
  {
    auto o = *this;
    o.k = (o.k + 1) & 3;
    return o;
  }
  Pt unitDir() { return (next().pt() - pt()).unit(); }

  template <class T>
  static std::vector<Pt> getNearbyObstaclePoints(T &map, Pt p, int d)
  {
    std::vector<Pt> os;

    int x = MAP_SIZE - p.y * 2, y = p.x * 2;
    for (int i = std::max(-1, x - d); i <= MAP_SIZE && i <= x + d; i++)
    {
      for (int j = std::max(-1, y - d); j <= MAP_SIZE && j <= y + d; j++)
      {
        if (i < 0 || j < 0 || i == MAP_SIZE || j == MAP_SIZE ||
            map[i][j] == '#')
        {
          D u = j * 0.5, v = (MAP_SIZE - 1 - i) * 0.5;
          os.emplace_back(u, v);
        }
      }
    }

    return os;
  }

  template <class T>
  static std::vector<std::pair<Pt, Pt>>
  getNearbyObstaclePointsToSegment(T &map, Pt p, int d)
  {
    std::vector<std::pair<Pt, Pt>> os;

    int x = MAP_SIZE - p.y * 2, y = p.x * 2;
    for (int i = std::max(-1, x - d); i <= MAP_SIZE && i <= x + d; i++)
    {
      for (int j = std::max(-1, y - d); j <= MAP_SIZE && j <= y + d; j++)
      {
        if (i < 0 || j < 0 || i == MAP_SIZE || j == MAP_SIZE ||
            map[i][j] == '#')
        {
          D u = j * 0.5, v = (MAP_SIZE - 1 - i) * 0.5;

          os.push_back({Pt(u, v),
                        Pt(u + 0.5, v)});
          os.push_back({Pt(u, v),
                        Pt(u, v + 0.5)});
          os.push_back({Pt(u + 0.5, v + 0.5),
                        Pt(u + 0.5, v)});
          os.push_back({Pt(u + 0.5, v + 0.5),
                        Pt(u, v + 0.5)});
        }
      }
    }

    return os;
  }

  template <class T>
  static std::vector<Obstacle> getNearbyObstacles(T &map, Pt p, int d)
  {

    std::vector<Obstacle> os;
    for (auto v : getNearbyObstaclePoints(map, p, d))
    {
      for (int i = 0; i < 4; i++)
      {
        os.push_back({v, i});
      }
    }

    return os;
  }

  // 判断线段 (a, b) 是否与障碍的距离不超过 radius
  static bool checkOS(const Pt &o0, const Pt &a, const Pt &b, D radius)
  {
    radius -= 0.015;
    Pt o1(o0.x + 0.5, o0.y), o2(o1.x, o1.y + 0.5), o3(o0.x, o2.y);
    return checkSS(o0, o1, a, b) || checkSS(o1, o2, a, b) ||
           checkSS(o2, o3, a, b) || checkSS(o3, o0, a, b) ||
           distPS(o0, a, b) <= radius ||
           distPS(o1, a, b) <= radius ||
           distPS(o2, a, b) <= radius ||
           distPS(o3, a, b) <= radius;
  }
};
