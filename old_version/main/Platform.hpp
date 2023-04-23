#pragma once
#include "Geometry.hpp"

class Platform {
public:
  const int id;
  const int type;
  const int need_time; // 生产需要的时间
  const int need_buy;
  const int sell;     // 销售物品的类型
  const int capacity; // 最多可以存多少销售物品
  const Point p;
  int prod_time; // 生产剩余时间
  int alreay_buy;
  int will_buy;
  int will_sell_num;
  int num; // 当前存了多少销售物
  int visit_robot_id;
  std::vector<int> apply_visit_list;
  bool active = true;

  std::vector<int> validAreas;
  Platform(int id, int type, int need_time, int need_buy, int sell,
           int capacity, double x, double y)
      : id(id), type(type), need_time(need_time), need_buy(need_buy),
        sell(sell), capacity(capacity), p(Point(x, y)) {
    prod_time = -1;
    alreay_buy = 0;
    num = 0;

    will_sell_num = 0;
    will_buy = 0;
    visit_robot_id = -1;
  }
  void readState();
  bool buyitem(int);
  int sellitem();
  int getnextime();
  bool can_buy(int);
  bool is_wait();
  void apply_visit(int);
  void process_apply_visit_list();
  void fresh_visit();
};