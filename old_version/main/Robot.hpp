#pragma once
#include "Geometry.hpp"
#include "obstacle.hpp"

class Robot
{
public:
  const int id;
  int state; // 0 空闲 1 在买的路上 2在卖的路上
  int buy;   // 机器人去哪个工作台买
  int buyPlatArea;
  int sell; // 机器人去哪个工作台卖
  int pid;  // 当前所在工作台id
  int item; // 携带物品类型
  Point p;
  Point next_to;
  Point preferredVelocity;
  double radius;
  double angle;
  double line_speed;
  Point v; // 速度
  double angle_speed;
  double tf; // 时间系数
  double cf; // 碰撞系数
  double rotangle;
  int rotframe;
  double max_angle_acc;
  double max_line_acc;
  bool wait;
  std::queue<std::pair<int, int>> tasks;
  Robot(int id, double x, double y) : id(id)
  {
    state = 0;
    buy = sell = pid = -1;
    buyPlatArea = 0;
    item = 0;
    radius = 0.45;
    angle = 0;
    line_speed = 0;
    v = Point(0, 0);
    angle_speed = 0;
    tf = cf = 0;

    this->p.x = x;
    this->p.y = y;
    rotangle = rotframe = 0;
  }
  void readState();
  void nextask();
  void fresh();
  int get_target();
  bool in_target();
  std::pair<D, D> getNewArgs(const std::vector<Robot> &,
                             const std::vector<Obstacle> &);
};