#pragma once
#include <bits/stdc++.h>
// using namespace std;
// mt19937 gen(2998388);
// // 随机生成[l,r]之间的实数
// double rddoulbe(double l, double r) { return
// uniform_real_distribution<double>(l, r)(gen); }
// // 随机生成[l,r]之间的整数
// int rdint(int l, int r) { return uniform_int_distribution<int>(l, r)(gen); }

using LL = long long;
using PII = std::pair<int, int>;

template <class T, class U> inline bool smin(T &x, const U &y) {
  return y < x ? x = y, 1 : 0;
}
template <class T, class U> inline bool smax(T &x, const U &y) {
  return x < y ? x = y, 1 : 0;
}

class TimeCounter {
private:
  int64_t beginTime;
  static inline int64_t getCurrentMillisecs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }

public:
  TimeCounter() { beginTime = getCurrentMillisecs(); }
  int64_t getTime() { return getCurrentMillisecs() - beginTime; };
};

// 计算时间损失
inline double time_loss(int x) {
  double minRate = 0.8;
  double maxX = 9000;
  return minRate + (1 - minRate) * (1 - sqrt(1 - pow(1 - 1.0 * x / maxX, 2)));
}

class Logger {
public:
  bool active = false;
  std::fstream out;
  ~Logger() {
    if (active)
      out.close();
  }
  void Load(std::string file_name, bool active = true) {
    this->active = active;
    if (this->active)
      out.open(file_name, std::fstream::out | std::ios_base::trunc);
  }
  template <class T, class... U> inline void write(T &&x, U &&...y) {
    out << x;
    ((out << " " << y), ...);
    out << "\n";
  }
};

template <class T, class... U> inline void dbg(T &&x, U &&...y) {
  std::cerr << x;
  ((std::cerr << " " << y), ...);
  std::cerr << "\n";
}

inline void myAssert(bool expr) {
  if (!expr) {
    std::cout << 1 / 0 << "\n";
  }
}