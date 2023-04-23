#pragma once
#include "utils.hpp"
using namespace std;

constexpr D EPS = 1e-4;
inline int sgn(D x) { return (x > EPS) - (x < -EPS); }
inline int cmp(D x, D y) { return sgn(x - y); }
inline D sqr(D x) { return x * x; }
constexpr long double PI =
    3.14159265358979323846264338327950288419716939937510L;

inline D normAngle(D a) {
  while (a >= PI)
    a -= 2 * PI;
  while (a < -PI)
    a += 2 * PI;
  return a;
}

struct Pt {
  D x, y;
  void read() { std::cin >> x >> y; }

  explicit Pt(D x = 0, D y = 0) : x(x), y(y) {}
  Pt operator-() const { return Pt(-x, -y); }
  Pt operator+(const Pt &r) const { return Pt(x + r.x, y + r.y); }
  Pt operator-(const Pt &r) const { return Pt(x - r.x, y - r.y); }
  Pt operator*(D r) const { return Pt(x * r, y * r); }
  Pt operator/(D r) const { return Pt(x / r, y / r); }
  bool operator<(const Pt &r) const { return x < r.x || x == r.x && y < r.y; }
  bool operator==(const Pt &r) const { return !cmp(x, r.x) && !cmp(y, r.y); }

  // D operator^(const Pt &r) const { return x * r.x + y * r.y; } // dot
  // D operator*(const Pt &r) const { return x * r.y - y * r.x; } // cross
  friend D dot(const Pt &a, const Pt &b) { return a.x * b.x + a.y * b.y; }
  D cross(Pt a, Pt b) const {
    return (a.x - x) * (b.y - y) - (a.y - y) * (b.x - x);
  }

  D len2() const { return x * x + y * y; }
  D len() const { return std::sqrt(len2()); }
  D dis2(const Pt &r) const { return (r - *this).len2(); }
  D dis(const Pt &r) const { return (r - *this).len(); }

  Pt rotate(D r) const {
    return Pt(x * cos(r) - y * sin(r), x * sin(r) + y * cos(r));
  }

  Pt unit() const { return *this / len(); }
  Pt perp() const { return Pt(-y, x); }
  Pt norm() const { return perp().unit(); }

  Pt proj(Pt a, Pt b) const {
    Pt d = b - a;
    return a + d * ((dot(d, *this - a)) / d.len2());
  }
  Pt refl(Pt a, Pt b) const { return proj(a, b) * 2 - *this; }

  bool onSeg(Pt a, Pt b) const {
    return !sgn(cross(a, b)) && sgn(dot(*this - a, *this - b)) <= 0;
  }
  int side(Pt a, Pt b) const { return sgn(a.cross(b, *this)); }
  int pos() const { return y > 0 || y == 0 && x > 0; }

  friend std::ostream &operator<<(std::ostream &o, const Pt &p) {
    return o << "(" << p.x << ", " << p.y << ")";
  }

  D angleTo(const Pt &to) { return std::atan2(to.y - y, to.x - x); }

  std::string str() {
    return "(" + std::to_string(x) + "," + std::to_string(y) + ")";
  }
};

using Point = Pt;

inline Pt operator*(D a, const Pt &b) { return b * a; }
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

struct Line {
  Pt p, v;
  bool operator<(const Line &r) const {
    int k = v.pos() - r.v.pos();
    return k < 0 || k == 0 && cross(v, r.v) > 0;
  }
};

inline bool checkPointArea(int k, Pt p) {
  const D E = 0.25;
  if (k == 0) {
    return true;
  }
  if (k == 1) {
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

// inline double GetCross(Point &p1, Point &p2, Point &p) {
//   return (p2.x - p1.x) * (p.y - p1.y) - (p.x - p1.x) * (p2.y - p1.y);
// }
// struct Rectangle {
//   Pt p1, p2, p3, p4;
//   /*
//   参考：https://blog.csdn.net/weixin_43619346/article/details/107513919
//   p1,p2,p3,p4逆时针
//   */
//   bool isIn(Point &p) // 点是否在矩形内
//   {
//     return GetCross(p1, p2, p) * GetCross(p3, p4, p) >= 0 &&
//            GetCross(p2, p3, p) * GetCross(p4, p1, p) >= 0;
//   }
// };
struct Circle {
  Pt p;
  double r;
  bool isIn(Point &op) { return p.dis(op) <= r; }
};

inline double getRadius(Pt a, Pt b, Pt c) {
  Pt u = c - a, v = b - a;
  double k = cross(u, v) * 2;
  return u.len() * v.len() * (b - c).len() / std::fabs(k); // a = 2r sinA
}
inline Pt getCenter(Pt a, Pt b, Pt c) {
  Pt u = c - a, v = b - a;
  double k = cross(u, v) * 2;
  return a + (u * v.len2() - v * u.len2()).perp() / k;
}