#pragma once
#include <bits/stdc++.h>
#include "Tools.hpp"

using namespace std;
using D = double;
const double eps = 1e-4;
const double pi = acos(-1);
const double EPS = 1e-4;
constexpr long double PI =
    3.14159265358979323846264338327950288419716939937510L;
inline D sqr(D x) { return x * x; }
inline int sgn(D x) { return (x > EPS) - (x < -EPS); }
inline int cmp(D x, D y) { return sgn(x - y); }
// 角度正则化到[-pi,pi]之间
inline double norm_angle(double x) {
  while (x >= pi)
    x -= 2 * pi;
  while (x < -pi)
    x += 2 * pi;
  return x;
}
template <typename T> class point {
public:
  T x, y;
  int id = -1;
  explicit point(T x = T(), T y = T()) : x(x), y(y) {}
  bool operator==(const point &a) const {
    return (abs(x - a.x) <= eps && abs(y - a.y) <= eps);
  }
  point operator+(const point &a) const { return point{x + a.x, y + a.y}; }
  point operator-(const point &a) const { return point{x - a.x, y - a.y}; }
  point operator-() const { return point{-x, -y}; }
  point operator*(const T k) const { return point{k * x, k * y}; }
  point operator/(const T k) const { return point{x / k, y / k}; }
  T operator*(const point &a) const { return x * a.x + y * a.y; } // Dot
  T operator^(const point &a) const { return x * a.y - y * a.x; } // Cross
  double angleto(point &to) { return atan2(to.y - y, to.x - x); }
  bool operator<(const point &a) const {
    if (abs(x - a.x) <= eps)
      return y < a.y - eps;
    return x < a.x - eps;
  }
  bool is_par(const point &a) const { return abs((*this) ^ a) <= eps; }
  bool is_ver(const point &a) const { return abs((*this) * a) <= eps; }
  int toleft(const point &a) const {
    auto t = (*this) ^ a;
    return (t > eps) - (t < -eps);
  }
  //<0在直线左边，=0在直线上，>0在直线右边
  T len2() const { return (*this) * (*this); }
  T dis2(const point &a) const { return (a - (*this)).len2(); }
  double len() const { return sqrt(len2()); }
  double dis(const point &a) const { return (a - (*this)).len(); }
  double ang(const point &a) const {
    return acos(((*this) * a) / (this->len() * a.len()));
  }
  point rot(const double rad) const {
    return {x * cos(rad) - y * sin(rad), x * sin(rad) + y * cos(rad)};
  }
  point unit() const { return *this / len(); }
  point perp() const { return point(-y, x); }
  point norm() const { return perp().unit(); }
  string str() { return " (" + to_string(x) + "," + to_string(y) + ")"; }
  T cross(const point &a, const point &b) const {
    return (a.x - x) * (b.y - y) - (a.y - y) * (b.x - x);
  }
};
template <typename T> class line {
public:
  point<T> p, v; // p+tv
  bool operator==(const line &a) const {
    return (v.is_par(a.v) && v.is_par(p - a.p));
  }
  bool is_par(const line &a) const {
    return (v.is_par(a.v) && !v.is_par(p - a.p));
  }
  bool is_ver(const line &a) const { return v.is_ver(a.v); }
  bool is_on(const point<T> &a) const { return v.is_par(a - p); }
  int toleft(const point<T> &a) const { return v.toleft(a - p); }
  point<T> inter(const line &a) const {
    return p + v * ((a.v ^ (p - a.p)) / (v ^ a.v));
  } // 交点
  double dis(const point<T> &a) const { return abs(v ^ (a - p)) / v.len(); }
  point<T> proj(const point<T> &a) const {
    return p + v * ((v * (a - p)) / (v * v));
  } // 点到直线的投影
};
template <typename T> struct segment {
  point<T> a, b;
  bool operator<(const segment &s) const {
    return make_pair(a, b) < make_pair(s.a, s.b);
  }
  int is_on(const point<T> &p) const {
    if (p == a || p == b)
      return -1;
    return (p - a).toleft(p - b) == 0 && (p - a) * (p - b) < -eps;
  }
  int is_inter(const line<T> &l) const {
    if (l.toleft(a) == 0 || l.toleft(b) == 0)
      return -1;
    return l.toleft(a) != l.toleft(b);
  }
  int is_inter(const segment<T> &s) const {
    if (is_on(s.a) || is_on(s.b) || s.is_on(a) || s.is_on(b))
      return -1;
    const line<T> l{a, b - a}, ls{s.a, s.b - s.a};
    return l.toleft(s.a) * l.toleft(s.b) == -1 &&
           ls.toleft(a) * ls.toleft(b) == -1;
  }
  long double dis(const point<T> &p) const {
    if ((p - a) * (b - a) < -eps || (p - b) * (a - b) < -eps)
      return min(p.dis(a), p.dis(b));
    const line<T> l{a, b - a};
    return l.dis(p);
  }
  long double dis(const segment<T> &s) const {
    if (is_inter(s))
      return 0;
    return min({dis(s.a), dis(s.b), s.dis(a), s.dis(b)});
  }
};
using Point = point<double>;
using Pt = point<double>;
using Line = line<double>;
using Segment = segment<double>;

inline Pt operator*(D a, const Pt &b) { return b * a; }
inline D dot(const Pt &a, const Pt &b) { return a.x * b.x + a.y * b.y; }
inline D cross(const Pt &a, const Pt &b) { return a.x * b.y - a.y * b.x; }

inline Pt getLL(Pt a, Pt b, Pt c, Pt d) {
  D p = c.cross(b, d), q = c.cross(d, a);
  return (a * p + b * q) / (p + q);
}
inline bool checkSS(Pt a, Pt b, Pt c, Pt d) {
  D oa = c.cross(d, a), ob = c.cross(d, b), oc = a.cross(b, c),
    od = a.cross(b, d);
  return sgn(oa) * sgn(ob) < 0 && sgn(oc) * sgn(od) < 0;
}
inline D distPL(Pt p, Pt a, Pt b) {
  return fabs(p.cross(a, b)) / (b - a).len();
}
inline D distPS(Pt p, Pt a, Pt b) {
  if (a == b)
    return (a - p).len();
  D d = (b - a).len2(), t = std::min(d, std::max(0.0, dot(b - a, p - a)));
  return ((p - a) * d - (b - a) * t).len() / d;
}
inline D distSS(Pt a, Pt b, Pt c, Pt d) {
  return !checkSS(a, b, c, d) ? std::min({distPS(a, c, d), distPS(b, c, d),
                                          distPS(c, a, b), distPS(d, a, b)})
                              : 0.0;
}

inline bool checkPointArea(int k, Pt p) {
  // return true;
  const D E = 0.15;
  if (k == 0) {
    return true;
  } if (k == 1) {
    return p.x < -E;
  } else if (k == 2) {
    return p.x > E;
  } else if (k == 3) {
    return p.y < -E;
  } else if (k == 4) {
    return p.y > E;
  } else {
    myAssert(false);
    return false;
  }
}