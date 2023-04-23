#pragma once
#include <bits/stdc++.h>

using LL = long long;
using PII = std::pair<int, int>;
using D = double;

// class Test {
//  private:
//   Test() {}

//  public:
//   Test(const Test &) = delete;
//   Test &operator=(const Test &) = delete;
//   Test(Test &&) = delete;
//   Test &operator=(Test &&) = delete;

//   static auto &instance() {
//     static Test t;
//     return t;
//   }
// };

class Random {
private:
  Random() : rng(124314) {}

public:
  std::mt19937_64 rng;
  Random(const Random &) = delete;
  Random &operator=(const Random &) = delete;
  Random(Random &&) = delete;
  Random &operator=(Random &&) = delete;

  static auto &instance() {
    static Random t;
    return t;
  }

  static auto randInt(int l, int r) {
    return std::uniform_int_distribution<int>(l, r)(instance().rng);
  }

  static auto randLL(LL l, LL r) {
    return std::uniform_int_distribution<int>(l, r)(instance().rng);
  }

  static auto randDouble(double l, double r) {
    return std::uniform_real_distribution<double>(l, r)(instance().rng);
  }
};

template <class T, class U> inline bool smin(T &x, const U &y) {
  return y < x ? x = y, 1 : 0;
}
template <class T, class U> inline bool smax(T &x, const U &y) {
  return x < y ? x = y, 1 : 0;
}

inline double getTime() { return (double)clock() / CLOCKS_PER_SEC; }

// 计算时间损失
inline double calTimeLoss(int x) {
  double minRate = 0.8;
  double maxX = 9000;
  return minRate + (1 - minRate) * (1 - sqrt(1 - pow(1 - 1.0 * x / maxX, 2)));
}

template <class T, class... U> inline void dbg(T &&x, U &&...y) {
  std::cerr << x;
  ((std::cerr << " " << y), ...);
  std::cerr << "\n";
}

class Logger {
private:
  Logger() {}

public:
  Logger(const Logger &) = delete;
  Logger &operator=(const Logger &) = delete;
  Logger(Logger &&) = delete;
  Logger &operator=(Logger &&) = delete;
  static auto &instance() {
    static Logger t;
    return t;
  }
  std::fstream out;
  ~Logger() { out.close(); }
  void load(std::string file_name) {
    out.open(file_name, std::fstream::out | std::ios_base::trunc);
  }
  template <class T, class... U> inline void write(T &&x, U &&...y) {
    out << x;
    ((out << " " << y), ...);
    out << "\n";
    dbg(x, y...);
  }
};

inline void myAssert(bool expr) {
  if (!expr) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdiv-by-zero"
    std::cout << 1 / 0 << "\n";
#pragma GCC diagnostic pop
  }
}