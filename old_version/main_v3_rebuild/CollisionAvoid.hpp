#pragma once
#include "Platform.hpp"
#include "geo.hpp"
#include "Map.hpp"

struct VelocitySelecter {
  using VL = std::vector<Line>;

  D rng;
  Pt ans;
  VelocitySelecter(D r) : rng(r), ans() {}

  int slv_normal(const VL &l, const Pt &tv, bool flag) {
    ans = tv;
    if (flag) {
      ans = ans * rng;
    } else if (tv.len2() > sqr(rng)) {
      ans = ans.unit() * rng;
    }

    for (int k = 0; k < l.size(); k++) {
      if (cross(l[k].v, l[k].p - ans) > 0.0) {
        D dp = -dot(l[k].p, l[k].v), val = sqr(dp) + sqr(rng) - l[k].p.len2();
        if (val < 0.0)
          return k;
        D vt = std::sqrt(val), tl = dp - vt, tr = dp + vt;
        for (int i = 0; i < k; i++) {
          D u = cross(l[k].v, l[i].v), v = cross(l[i].v, l[k].p - l[i].p);
          if (!sgn(u) && v < 0.0)
            return k;
          if (sgn(u)) {
            u >= 0 ? smin(tr, v / u) : smax(tl, v / u);
            if (tl > tr)
              return k;
          }
        }

        ans = l[k].p +
              l[k].v *
                  (flag ? (dot(tv, l[k].v) > 0 ? tr : tl)
                        : std::max(tl, std::min(tr, dot(l[k].v, tv - l[k].p))));
      }
    }
    return l.size();
  }

  void slv_extra(const VL &l, int k, int s) {
    D d = 0;

    for (int i = s; i < l.size(); i++) {
      if (cross(l[i].v, l[i].p - ans) > d) {
        VL pls(l.begin(), l.begin() + k);

        for (int j = k; j < i; j++) {
          D v = cross(l[i].v, l[j].v);

          if (!sgn(v) && dot(l[i].v, l[j].v) > 0.0)
            continue;
          pls.push_back(
              {sgn(v) ? l[i].p + l[i].v * (cross(l[j].v, l[i].p - l[j].p) / v)
                      : (l[i].p + l[j].p) * 0.5,
               (l[j].v - l[i].v).unit()});
        }

        Pt cur = ans;

        if (slv_normal(pls, l[i].v.perp(), true) < pls.size()) {
          ans = cur;
        }

        d = cross(l[i].v, l[i].p - ans);
      }
    }
  }
};

struct HalfplaneBuilder {
  const Robot &robot;
  const std::vector<Robot> &rs;
  const std::vector<Obstacle> &os;
  const D radius, timeHorizon, timeHorizonObst;
  std::vector<Line> lines;
  int k;

  HalfplaneBuilder(const Robot &robot, const std::vector<Robot> &rs,
                   const std::vector<Obstacle> &os, const D radius,
                   const D timeHorizon, const D timeHorizonObst)
      : robot(robot), rs(rs), os(os), radius(radius), timeHorizon(timeHorizon),
        timeHorizonObst(timeHorizonObst) {
    lines.clear();

    handleObstacles();

    k = lines.size();

    handleRobots();
  }

  Pt solve(D maxSpeed) {
    VelocitySelecter slv(maxSpeed);

    int s = slv.slv_normal(lines, robot.preferredVelocity, false);

    if (s < lines.size()) {
      slv.slv_extra(lines, k, s);
    }

    return slv.ans;
  }

  // D getArea(D maxSpeed) {
  //   return getAreaCP(Circle(0, 0, maxSpeed),
  //                    halfplaneIntersection(lines, maxSpeed));
  // }

  void handleObstacle(Obstacle o) {
    const D invT = 1.0 / TIME_HORIZON_OBSTACLE;
    auto oNext = o.next();
    Pt o1 = o.pt(), o2 = oNext.pt();
    Pt p1 = o1 - robot.p, p2 = o2 - robot.p;

    bool flag = false;
    for (auto &l : lines) {
      D z = invT * radius - EPS;
      if (cross(p1 * invT - l.p, l.v) >= z &&
          cross(p2 * invT - l.p, l.v) >= z) {
        flag = true;
        break;
      }
    }

    if (flag) {
      return;
    }

    const D dSq1 = p1.len2(), dSq2 = p2.len2(), rSq = sqr(radius);
    const Pt ov = o2 - o1;
    const D s = -dot(p1, ov) / ov.len2();
    const D dSqL = (p1 + ov * s).len2();

    if (s < 0.0f && dSq1 <= rSq) {
      lines.push_back({Pt(), p1.norm()});
      return;
    }

    if (s > 1.0f && dSq2 <= rSq) {
      if (cross(p2, oNext.unitDir()) >= 0) {
        lines.push_back({Pt(), p2.norm()});
      }
      return;
    }

    if (s >= 0.0f && s < 1.0f && dSqL <= rSq) {
      lines.push_back({Pt(), -o.unitDir()});
      return;
    }

    Pt l, r; // direction of two legs
    if (s < 0.0 && dSqL <= rSq) {
      o2 = o1, oNext = o;
      D leg1 = std::sqrt(dSq1 - rSq);
      l = Pt(p1.x * leg1 - p1.y * radius, p1.y * leg1 + p1.x * radius) / dSq1;
      r = Pt(p1.x * leg1 + p1.y * radius, p1.y * leg1 - p1.x * radius) / dSq1;
    } else if (s > 1.0 && dSqL <= rSq) {
      o1 = o2, o = oNext;
      D leg2 = std::sqrt(dSq2 - rSq);
      l = Pt(p2.x * leg2 - p2.y * radius, p2.y * leg2 + p2.x * radius) / dSq2;
      r = Pt(p2.x * leg2 + p2.y * radius, p2.y * leg2 - p2.x * radius) / dSq2;
    } else {
      D leg1 = std::sqrt(dSq1 - rSq);
      l = Pt(p1.x * leg1 - p1.y * radius, p1.y * leg1 + p1.x * radius) / dSq1;
      D leg2 = std::sqrt(dSq2 - rSq);
      r = Pt(p2.x * leg2 + p2.y * radius, p2.y * leg2 - p2.x * radius) / dSq2;
    }

    auto oPrev = o.prev();

    bool lf = false, rf = false;

    if (cross(l, -oPrev.unitDir()) >= 0) {
      l = -oPrev.unitDir();
      lf = true;
    }

    if (cross(r, oNext.unitDir()) <= 0) {
      r = oNext.unitDir();
      rf = true;
    }

    Pt lc = (o1 - robot.p) * invT, rc = (o2 - robot.p) * invT, cv = rc - lc;

    const D t = o1 == o2 ? 0.5 : dot(robot.v - lc, cv) / cv.len2();
    const D tl = dot(robot.v - lc, l), tr = dot(robot.v - rc, r);

    if (t < 0.0 && tl < 0.0 || o1 == o2 && tl < 0.0 && tr < 0.0) {
      const Pt uw = (lc - robot.v).unit();
      lines.push_back({lc - uw * (invT * radius), uw.perp()});
      return;
    }

    if (t > 1.0 && tr < 0.0) {
      const Pt uw = (rc - robot.v).unit();
      lines.push_back({rc - uw * (invT * radius), uw.perp()});
      return;
    }

    const D dc = t < 0.0 || t > 1.0 || o1 == o2
                     ? std::numeric_limits<D>::max()
                     : (robot.v - (lc + cv * t)).len2();
    const D dl = tl < 0.0 ? std::numeric_limits<D>::max()
                          : (robot.v - (lc + l * tl)).len2();
    const D dr = tr < 0.0 ? std::numeric_limits<D>::max()
                          : (robot.v - (rc + r * tr)).len2();

    if (dc <= dl && dc <= dr) {
      Pt v = -o.unitDir(), p = lc + v.perp() * (invT * radius);
      lines.push_back({p, v});
      return;
    }

    if (dl <= dr) {
      if (!lf) {
        lines.push_back({lc + l.perp() * (invT * radius), l});
      }
      return;
    }

    if (!rf) {
      Pt v = -r, p = rc + v.perp() * (invT * radius);
      lines.push_back({p, v});
    }
  }

  void handleObstacles() {
    for (auto o : os) {
      handleObstacle(o);
    }
  }

  void handleRobots() {
    const D invT = 1.0 / TIME_HORIZON;

    for (auto &o : rs) {
      if (o.id == robot.id)
        continue;
      Pt rp = o.p - robot.p, rv = robot.v - o.v, u, v;
      D d2 = rp.len2(), sr = radius + R[o.item > 0] + 0.05, sr2 = sqr(sr);

      if (d2 > sr2) {

        Pt w = rp * invT - rv;
        D dp = dot(w, rp);

        if (dp > 0.0 && sqr(dp) > sr2 * w.len2()) {
          v = w.norm();
          u = w - w.unit() * (sr * invT);
        } else {
          D leg = std::sqrt(d2 - sr2);
          v = cross(rp, w) < 0.0
                  ? Pt(rp.x * leg - rp.y * sr, rp.y * leg + rp.x * sr)
                  : Pt(-rp.x * leg - rp.y * sr, rp.x * sr - rp.y * leg);
          v = v / d2;
          u = v * dot(rv, v) - rv;
        }
      } else {
        D invT = 1.0 / TIME_STEP;
        Pt w = rp * invT - rv;
        v = w.norm();
        u = w - w.unit() * (sr * invT);
      }

      lines.push_back({robot.v + u * 0.5, v});
    }
  }
};

inline std::pair<D, D> getNewArgs(const Robot &rob, const std::vector<Robot> &rs,
                           const std::vector<Obstacle> &os) {
  Pt optV =
      HalfplaneBuilder(rob, rs, os, rob.radius + 0.02, TIME_HORIZON,
                       TIME_HORIZON_OBSTACLE)
          .solve(std::fmin(MAX_FORWARD_SPEED[Map::instance().role],
                           rob.v.len() + MAX_DELTA_lineSpeed_PER_FRAME[rob.item > 0]));
  D angleDist = normAngle(atan2(optV.y, optV.x) - rob.angle);
  D absAngle = std::fabs(angleDist);
  D maxW = std::min(absAngle / TIME_STEP, MAX_ROTATE_SPEED) * sgn(angleDist);
  D maxV = optV.len();
  D eV, eW;

  eW = maxW;
  if (absAngle > PI / 8) {
    eV = maxV * 0.1;
  } else {
    // eV = maxV * std::cos(absAngle);
    eV = maxV;
    eW = maxW * std::sin(absAngle);
  }

  return {eV, eW};
}