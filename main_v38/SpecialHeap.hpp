#pragma once
#include "geo.hpp"
#include "constants.hpp"

struct QInfo {
  D v;
  int x;
  bool operator<(const QInfo &r) const {
    return r.v < v;
  }
  QInfo(D v, int x) : v(v), x(x) {}
};

template <class T>
struct SpecialHeap {
  static constexpr int N = 4e5;

  int o, cur[N], min[N * 2], s[N], t;
  T val[N], info[N][2];
  int par[N];

  static auto &instance(int k) {
    static SpecialHeap<T> sh[5];
    return sh[k];
  }

  void clear() {
    o = 0;
    while (t) {
      cur[s[--t]] = 0;
    }
  }
  void set(int i, int x) {
    min[i] = x;
    cur[x] = i;
  }
  
  bool isOpen(int x) {
    return !cur[x];
  }

  bool isClosed(int x) {
    return cur[x] == -1;
  }

  int get(int i, int j) {
    return val[i] < val[j] ? i : j;
  }

  void update(int i) {
    const int x = min[i];
    const T d = val[x];
    while ((i >>= 1) && val[min[i]] >= d) min[i] = x;
  }

  void insert(int x) {
    s[t++] = x;
    set(o * 2 + 1, x);
    if (o) {
      set(o * 2, min[o]);
      update(o * 2 + 1);
    }
    o++;
  }

  void decrease(int x) {
    update(cur[x]);
  }

  void pop() {
    int t = top(), i = cur[t];
    if (--o) {
      if (o == (i >> 1)) {
        set(o, min[i ^ 1]);
        i = o;
      } else {
        set(i, min[o] != min[o * 2] ? min[o * 2] : min[o * 2 + 1]);
        cur[min[o]] = o;
      }
      for (int x; i != 1; min[i >>= 1] = x)
        x = get(min[i], min[i ^ 1]);
    }
    cur[t] = -1;
  }

  int top() {
    return min[1];
  }

  bool empty() {
    return !o;
  }
};

