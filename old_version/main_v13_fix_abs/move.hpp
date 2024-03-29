#pragma once
#include "Map.hpp"
#include "Platform.hpp"
#include "geo.hpp"

using namespace std;

struct VelocitySelecter
{
  using VL = std::vector<Line>;

  D rng;
  Pt ans;
  VelocitySelecter(D r) : rng(r), ans() {}

  int solveNormal(const VL &l, const Pt &tv, bool flag)
  {
    ans = tv;
    if (flag)
    {
      ans = ans * rng;
    }
    else if (tv.len2() > sqr(rng))
    {
      ans = ans.unit() * rng;
    }

    for (int k = 0; k < l.size(); k++)
    {
      if (cross(l[k].v, l[k].p - ans) > 0.0)
      {
        D dp = -dot(l[k].p, l[k].v), val = sqr(dp) + sqr(rng) - l[k].p.len2();
        if (val < 0.0)
          return k;
        D vt = std::sqrt(val), tl = dp - vt, tr = dp + vt;
        for (int i = 0; i < k; i++)
        {
          D u = cross(l[k].v, l[i].v), v = cross(l[i].v, l[k].p - l[i].p);
          if (!sgn(u) && v < 0.0)
            return k;
          if (sgn(u))
          {
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

  void solveExtra(const VL &l, int k, int s)
  {
    D d = 0;

    for (int i = s; i < l.size(); i++)
    {
      if (cross(l[i].v, l[i].p - ans) > d)
      {
        VL pls(l.begin(), l.begin() + k);

        for (int j = k; j < i; j++)
        {
          D v = cross(l[i].v, l[j].v);

          if (!sgn(v) && dot(l[i].v, l[j].v) > 0.0)
            continue;
          pls.push_back(
              {sgn(v) ? l[i].p + l[i].v * (cross(l[j].v, l[i].p - l[j].p) / v)
                      : (l[i].p + l[j].p) * 0.5,
               (l[j].v - l[i].v).unit()});
        }

        Pt cur = ans;

        if (solveNormal(pls, l[i].v.perp(), true) < pls.size())
        {
          ans = cur;
        }

        d = cross(l[i].v, l[i].p - ans);
      }
    }
  }
};

struct HalfplaneBuilder
{
  const Robot &robot;
  const std::vector<Robot> &rs;
  const std::vector<Obstacle> &os;
  const D radius, th, thObst;
  std::vector<Line> lines;
  int k;

  bool robotCollision, obstCollision, normalNoSolution;

  HalfplaneBuilder(const Robot &robot, const std::vector<Robot> &rs,
                   const std::vector<Obstacle> &os, const D radius, const D th,
                   const D thObst)
      : robot(robot), rs(rs), os(os), radius(radius), th(th), thObst(thObst),
        lines(), k(), robotCollision(false), obstCollision(false),
        normalNoSolution(false)
  {
    lines.clear();

    handleObstacles();

    k = lines.size();

    handleRobots();
  }

  Pt solve(D maxSpeed)
  {
    VelocitySelecter vs(maxSpeed);

    int s = vs.solveNormal(lines, robot.preferredVelocity, false);

    if (s < lines.size())
    {
      normalNoSolution = true;
      vs.solveExtra(lines, k, s);
    }

    return vs.ans;
  }

  void handleObstacle(Obstacle o)
  {
    const D invT = 1.0 / thObst;
    auto oNext = o.next();
    Pt o1 = o.pt(), o2 = oNext.pt();
    Pt p1 = o1 - robot.p, p2 = o2 - robot.p;

    for (auto &l : lines)
    {
      D z = invT * radius - EPS;
      if (cross(p1 * invT - l.p, l.v) >= z &&
          cross(p2 * invT - l.p, l.v) >= z)
      {
        return;
      }
    }

    const D dSq1 = p1.len2(), dSq2 = p2.len2(), rSq = sqr(radius);
    const Pt ov = o2 - o1;
    const D s = -dot(p1, ov) / ov.len2();
    const D dSqL = (p1 + ov * s).len2();

    if (s < 0.0 && dSq1 <= rSq)
    {
      obstCollision = true;
      lines.push_back({Pt(), p1.norm()});
      return;
    }

    if (s > 1.0 && dSq2 <= rSq)
    {
      obstCollision = true;
      if (cross(p2, oNext.unitDir()) >= 0)
      {
        lines.push_back({Pt(), p2.norm()});
      }
      return;
    }

    if (s >= 0.0 && s < 1.0 && dSqL <= rSq)
    {
      obstCollision = true;
      lines.push_back({Pt(), -o.unitDir()});
      return;
    }

    Pt l, r; // direction of two legs
    if (s < 0.0 && dSqL <= rSq)
    {
      o2 = o1, oNext = o;
      D leg1 = std::sqrt(dSq1 - rSq);
      l = Pt(p1.x * leg1 - p1.y * radius, p1.y * leg1 + p1.x * radius) / dSq1;
      r = Pt(p1.x * leg1 + p1.y * radius, p1.y * leg1 - p1.x * radius) / dSq1;
    }
    else if (s > 1.0 && dSqL <= rSq)
    {
      o1 = o2, o = oNext;
      D leg2 = std::sqrt(dSq2 - rSq);
      l = Pt(p2.x * leg2 - p2.y * radius, p2.y * leg2 + p2.x * radius) / dSq2;
      r = Pt(p2.x * leg2 + p2.y * radius, p2.y * leg2 - p2.x * radius) / dSq2;
    }
    else
    {
      D leg1 = std::sqrt(dSq1 - rSq);
      l = Pt(p1.x * leg1 - p1.y * radius, p1.y * leg1 + p1.x * radius) / dSq1;
      D leg2 = std::sqrt(dSq2 - rSq);
      r = Pt(p2.x * leg2 + p2.y * radius, p2.y * leg2 - p2.x * radius) / dSq2;
    }

    auto oPrev = o.prev();

    bool lf = false, rf = false;

    if (cross(l, -oPrev.unitDir()) >= 0)
    {
      l = -oPrev.unitDir();
      lf = true;
    }

    if (cross(r, oNext.unitDir()) <= 0)
    {
      r = oNext.unitDir();
      rf = true;
    }

    Pt lc = (o1 - robot.p) * invT, rc = (o2 - robot.p) * invT, cv = rc - lc;

    const D t = o1 == o2 ? 0.5 : dot(robot.v - lc, cv) / cv.len2();
    const D tl = dot(robot.v - lc, l), tr = dot(robot.v - rc, r);

    if (t < 0.0 && tl < 0.0 || o1 == o2 && tl < 0.0 && tr < 0.0)
    {
      const Pt uw = (lc - robot.v).unit();
      lines.push_back({lc - uw * (invT * radius), uw.perp()});
      return;
    }

    if (t > 1.0 && tr < 0.0)
    {
      const Pt uw = (rc - robot.v).unit();
      lines.push_back({rc - uw * (invT * radius), uw.perp()});
      return;
    }

    const D dc = t < 0.0 || t > 1.0 || o1 == o2
                     ? std::numeric_limits<D>::infinity()
                     : (robot.v - (lc + cv * t)).len2();
    const D dl = tl < 0.0 ? std::numeric_limits<D>::infinity()
                          : (robot.v - (lc + l * tl)).len2();
    const D dr = tr < 0.0 ? std::numeric_limits<D>::infinity()
                          : (robot.v - (rc + r * tr)).len2();

    if (dc <= dl && dc <= dr)
    {
      Pt v = -o.unitDir(), p = lc + v.perp() * (invT * radius);
      lines.push_back({p, v});
      return;
    }

    if (dl <= dr)
    {
      if (!lf)
      {
        lines.push_back({lc + l.perp() * (invT * radius), l});
      }
      return;
    }

    if (!rf)
    {
      Pt v = -r, p = rc + v.perp() * (invT * radius);
      lines.push_back({p, v});
    }
  }

  void handleObstacles()
  {
    for (auto o : os)
    {
      handleObstacle(o);
    }
  }

  void handleRobots()
  {
    const D invT = 1.0 / th;

    for (auto &o : rs)
    {
      if (o.id == robot.id)
        continue;
      Pt rp = o.p - robot.p, rv = robot.v - o.v, u, v;
      D d2 = rp.len2(), sr = radius + o.radius + OTHER_EXR, sr2 = sqr(sr);

      if (d2 > sr2)
      {

        Pt w = rp * invT - rv;
        D dp = dot(w, rp);

        if (dp > 0.0 && sqr(dp) > sr2 * w.len2())
        {
          v = w.norm();
          u = w - w.unit() * (sr * invT);
        }
        else
        {
          D leg = std::sqrt(d2 - sr2);
          v = cross(rp, w) < 0.0
                  ? Pt(rp.x * leg - rp.y * sr, rp.y * leg + rp.x * sr)
                  : Pt(-rp.x * leg - rp.y * sr, rp.x * sr - rp.y * leg);
          v = v / d2;
          u = v * dot(rv, v) - rv;
        }
      }
      else
      {
        robotCollision = true;
        D invT = 1.0 / TIME_STEP;
        Pt w = rp * invT - rv;
        v = w.norm();
        u = w - w.unit() * (sr * invT);
      }

      lines.push_back({robot.v + u * 0.5, v});
    }
  }
};

inline std::pair<D, D> getArgs(D direction, Pt optV)
{
  D angleDist = normAngle(std::atan2(optV.y, optV.x) - direction);
  D absAngle = std::fabs(angleDist);
  D maxW =
      std::min(absAngle / TIME_STEP * 2, MAX_ROTATE_SPEED) * sgn(angleDist);
  D maxV = optV.len();
  D eV, eW;

  if (absAngle > PI / 8)
  {
    eV = maxV * 0.1;
    eW = maxW;
  }
  else
  {
    // eV = maxV * std::cos(absAngle);
    eV = maxV;
    eW = maxW * std::sin(absAngle);
  }

  return {eV, eW};
}

inline std::pair<D, D> getArgs(Robot &r, Pt optV, bool robotCollision,
                               bool obstCollision)
{
  D angleDist = normAngle(std::atan2(optV.y, optV.x) - r.angle);
  D absAngle = std::fabs(angleDist);
  D maxW =
      std::min(absAngle / TIME_STEP * 2, MAX_ROTATE_SPEED) * sgn(angleDist);
  D maxV = optV.len();
  D eV, eW;

  if (absAngle > PI / 8)
  {
    eV = absAngle > PI / 2 ? 0 : maxV * 0.1;
    eW = maxW;
  }
  else
  {
    eV = maxV;
    eW = maxW * std::sin(absAngle);
    // eW = robotCollision || obstCollision || absAngle > 0.08
    //          ? maxW * std::sin(absAngle)
    //          : 0;
    // if (r.item && (robotCollision || obstCollision))
    //   eV /= 3;
    // if (!r.item && robotCollision) {
    //   eV /= 2;
    // }
  }

  // if (r.p.dis(r.nextTo) < 5) {
  //   if (obstCollision && !robotCollision && !r.item) {
  //     smax(eV, 1.0);
  //   }

  //   // if (r.item >= 4 && r.p.dis(r.nextTo) < 0.6)
  //   //   eV /= 2;
  // }

  // if (robotCollision || obstCollision) {
  //   eW = maxW;
  // }
  return {eV, eW};
}

inline std::pair<D, D> getArgs2(Robot &r, Pt optV, bool robotCollision,
                                bool wallCollision)
{
  const D MIN_ANGLE = 0.08;
  // 根据newV设置机器人的控制指令
  D theta = normAngle(std::atan2(optV.y, optV.x) - r.angle);
  D absTheta = std::fabs(theta);
  const D maxW = MAX_ROTATE_SPEED * sgn(theta);
  D rot = 0.0;
  D v = optV.len();

  if (absTheta >= MIN_ANGLE)
  {
    // 如果朝向和目标点的夹角超过阈值，需要调整
    if (absTheta > PI / 3 && r.p.dis(r.nextTo) >= 5)
    {
      // 角度太大，距离足够远，全速扭转
      // 速度减小，防止漂移
      v = 0;
      rot = maxW;
    }
    else
    {
      if (r.p.dis(r.nextTo) < 5)
      {
        if (wallCollision && !robotCollision && !r.item)
        {
          smax(v, 1.0);
        }
        // 有物品且距离小(近墙)
        if (absTheta >= PI / 4)
        {
          v = std::min(1.0, v) * (absTheta < PI / 2 ? 1 : -1);
        }
        else
        {
          if (r.item >= 4 && r.p.dis(r.nextTo) < 1.0)
            v *= 0.2;
          else
            smin(v, 6 * cos(absTheta));
        }
      }

      if (robotCollision || wallCollision)
      {
        // 检测到碰撞全力调整
        rot = maxW;
      }
      else
      {
        rot = maxW * sin(absTheta); // 旋转速度随角度变小而变小
      }
    }
  }

  if (absTheta < MIN_ANGLE && (robotCollision || wallCollision))
  {
    /* 如果朝向和目标点的夹角很小，但有碰撞，进行微调 */
    rot = maxW * sin(absTheta);
  }

  return {v, rot};
}
inline void demoMove(Robot &rob)
{
  // if (rob.state == 0)
  // {
  //   rob.lineSpeed = rob.angleSpeed = 0;
  //   return;
  // }
  D angle_dis = normAngle(rob.p.angleTo(rob.nextTo) - rob.angle);
  const D maxW = (angle_dis > 0 ? MAX_ROTATE_SPEED : -MAX_ROTATE_SPEED);
  D maxSpeed = MAX_FORWARD_SPEED[Map::instance().role];

  if (abs(angle_dis) < 0.08)
  {
    rob.lineSpeed = maxSpeed;
    rob.angleSpeed = 0;
  }
  else if (abs(angle_dis) > PI / 8)
  {
    rob.lineSpeed = maxSpeed * 0.0;
    rob.angleSpeed = maxW;
  }
  else
  {
    rob.lineSpeed = maxSpeed * cos(abs(angle_dis));
    rob.angleSpeed = maxW * sin(abs(angle_dis));
  }
}

// inline void moveRobots() {
//   auto &mp = Map::instance();
//   auto &mpFoe = Map::instanceFoe();
//   auto &robots = mp.robots;

//   static std::vector<int> ids;
//   static std::vector<int> hPFrame;

//   if (ids.empty()) {
//     ids.resize(robots.size());
//     iota(ids.begin(), ids.end(), 0);
//   }
//   if (hPFrame.empty())
//     hPFrame.resize(robots.size());

//   stable_sort(ids.begin(), ids.end(), [&](int x, int y) {
//     return robots[x].item > robots[y].item ||
//            robots[x].item == robots[y].item && x < y;
//   });

//   // for (size_t i = 0; i < ids.size(); ++i) {
//   //   --hPFrame[ids[i]];
//   //   bool canMove = false;
//   //   auto &r = robots[ids[i]];
//   //   // 矩形写法
//   //   D mnDis = 1.4; // 参数，预测机器人能否往某个方向走mnDis米
//   //   for (D dir = -PI; dir <= PI; dir += PI / 30) {
//   //     bool flag = true;
//   //     Point direction(cos(dir), sin(dir));
//   //     Point nex = r.p + direction * mnDis;
//   //     D robR = r.radius - 0.01; // 参数，影响矩形判断精度
//   //     Rectangle rec = {r.p + direction.rotate(PI / 2) * robR,
//   //                      r.p + direction.rotate(-PI / 2) * robR,
//   //                      nex + direction.rotate(-PI / 2) * robR,
//   //                      nex + direction.rotate(PI / 2) * robR};
//   //     Circle cir = {nex, r.radius};
//   //     D angleDis = normAngle(r.p.angleTo(nex) - r.angle);
//   //     if (angleDis < 0)
//   //       angleDis += 2 * PI;
//   //     int mid = angleDis / PI * 180;
//   //     int range = 89; // 参数，枚举雷达的范围
//   //     for (int j = mid - range; j <= mid + range; ++j) {
//   //       D len = r.radar[(j % 360 + 360) % 360];
//   //       assert(len >= r.radius);
//   //       D tmpAngle = r.angle + 1.0 * j / 180 * PI;
//   //       Point obs = r.p + Point(cos(tmpAngle), sin(tmpAngle)) * len;
//   //       if (rec.isIn(obs) || cir.isIn(obs)) {
//   //         // dbg(r.p.str(), nex.str(), mnDis, direction.str());
//   //         // dbg(rec.p1.str(), rec.p2.str(), rec.p3.str(), rec.p4.str(),
//   //         // obs.str()); exit(0);
//   //         flag = false;
//   //         break;
//   //       }
//   //     }
//   //     if (flag) {
//   //       canMove = true;
//   //       break;
//   //     }
//   //   }
//   //   //  矩形写法结束

//   //   // bfs写法
//   //   // 参数，机器人在格子地图能走至少gridStep步，则没被其它机器人夹住
//   //   // int gridStep = 4;
//   //   // auto obs =
//   //   //     Obstacle::getNearbyObstaclePointsToSegment(mp.g, r.p, gridStep +
//   //   2);
//   //   // auto check = [&](int xx, int yy) {
//   //   //   const D checkEps = 0.02; // 参数，check精度
//   //   //   if (xx <= 0 || xx >= mp.gn - 1 || yy <= 0 || yy >= mp.gn - 1)
//   //   //     return false;
//   //   //   Point p(xx * mp.gstep, yy * mp.gstep);
//   //   //   for (auto &rb : robots)
//   //   //     if (rb.id != r.id && rb.p.dis(p) < r.radius + rb.radius -
//   //   checkEps)
//   //   //       return false;
//   //   //   for (auto &ob : obs)
//   //   //     if (distPS(p, ob.first, ob.second) < r.radius - checkEps)
//   //   //       return false;
//   //   //   return true;
//   //   // };

//   //   // bool checkOk = false;
//   //   // int x = r.p.x / mp.gstep, y = r.p.y / mp.gstep;
//   //   // for (int dx = 0; dx <= 1; ++dx)
//   //   //   for (int dy = 0; dy <= 1; ++dy)
//   //   //     if (check(x + dx, y + dy)) {
//   //   //       x += dx;
//   //   //       y += dy;
//   //   //       checkOk = true;
//   //   //       break;
//   //   //     }
//   //   // if (!checkOk)
//   //   //   canMove = false;
//   //   // else {
//   //   //   std::vector<std::vector<int>> dis(
//   //   //       2 * gridStep + 1, std::vector<int>(2 * gridStep + 1,
//   //   INT32_MAX));
//   //   //   dis[gridStep][gridStep] = 0;
//   //   //   std::queue<std::pair<int, int>> q;
//   //   //   q.push({x, y});
//   //   //   while (!q.empty()) {
//   //   //     int xx = q.front().first, yy = q.front().second;
//   //   //     q.pop();
//   //   //     Point p(xx * mp.gstep, yy * mp.gstep);
//   //   //     if (!check(xx, yy))
//   //   //       continue;
//   //   //     if (dis[xx - x + gridStep][yy - y + gridStep] >= gridStep) {
//   //   //       canMove = true;
//   //   //       break;
//   //   //     }
//   //   //     for (int dx = -1; dx <= 1; ++dx)
//   //   //       for (int dy = -1; dy <= 1; ++dy)
//   //   //         if (dx || dy) {
//   //   //           int xxx = xx + dx;
//   //   //           int yyy = yy + dy;
//   //   //           if (xxx >= x - gridStep && xxx <= x + gridStep &&
//   //   //               yyy >= y - gridStep && yyy <= y + gridStep &&
//   //   //               dis[xxx - x + gridStep][yyy - y + gridStep] >
//   //   //                   dis[xx - x + gridStep][yy - y + gridStep] + 1) {
//   //   //             dis[xxx - x + gridStep][yyy - y + gridStep] =
//   //   //                 dis[xx - x + gridStep][yy - y + gridStep] + 1;
//   //   //             q.push({xxx, yyy});
//   //   //           }
//   //   //         }
//   //   //   }
//   //   // }
//   //   // bfs写法结束

//   //   // dbg(ids[i], canMove);
//   // const int C = 15; // 参数，保持最高有限制至少C帧
//   //   if (!canMove)
//   //     hPFrame[ids[i]] = C;
//   //   if (hPFrame[ids[i]] > 0) {
//   //     int j = i;
//   //     while (j) {
//   //       std::swap(ids[j - 1], ids[j]);
//   //       --j;
//   //     }
//   //   }
//   // }

//   std::vector<std::pair<Pt, Pt>> segs;
//   for (int id : ids) {
//     auto &r = robots[id];
//     if (r.isTransporter()) {
//       int pid = r.getTarget();
//       r.nextTo = mp.getNextTo(r.p, pid, r.state == 1 ? r.buyPlatArea : 0,
//                               r.radius, segs);

//       Pt u = r.p, v = r.nextTo - u;
//       if (v.len() < 0.001) {
//         segs.push_back({u, u});
//       } else {
//         segs.push_back(
//             {u - v.unit() * r.radius,
//              u + v.unit() *
//                      (r.radius +
//                       std::min(v.len(),
//                                (r.v.len() +
//                                 MAX_DELTA_LINE_SPEED_PER_FRAME[r.item > 0]) /
//                                    FPS))});
//       }
//     }
//   }
//   for (int id : ids) {
//     auto &r = robots[id];
//     if (r.isAttacker()) {
//       // dbg(r.attackFoeId, r.attackPlatId);
//       if (r.attackFoeId !=
//           -1) { // 雷达能照射到敌方机器人，则不太容易挡自己机器人的路
//         auto &e = mp.foes[r.attackFoeId];
//         r.nextTo = e.pos + e.velocity / FPS * 0; // 参数，预估帧数
//       } else if (r.attackPlatId != -1) {
//         r.nextTo = mpFoe.getNextTo(r.p, r.attackPlatId, 0, r.radius, segs);
//       }
//     }
//   }

//   for (Platform &p : mp.plats)
//     p.freshVisit();
//   for (Robot &rob : robots) {
//     rob.fresh();
//     rob.wait = false;
//     int target = rob.getTarget();
//     if (target != -2 && mp.getEvalDis(rob) <= 6)
//       mp.plats[target].applyVisit(rob.id);
//   }
//   for (Platform &p : mp.plats)
//     p.processApplyVisitList();

//   // if (mp.map_id == 1) {
//   for (Robot &rob : robots) {
//     int target = rob.getTarget();
//     if (target != -2 && mp.getEvalDis(rob) <= 6 &&
//         mp.plats[target].visitRobotId != rob.id)
//       rob.wait = true;
//   }
//   // }

//   for (auto &r : robots) {
//     int target = r.getTarget();
//     D maxSpeed = MAX_FORWARD_SPEED[mp.role];
//     if (r.wait)
//       maxSpeed *= 0.05;
//     Point p = r.nextTo - r.p;
//     D d = p.len();
//     if (target >= 0)
//       d = r.p.dis(mp.plats[target].p);
//     if (p.len() <= 1e-8) {
//       r.preferredVelocity = Point(0, 0);
//     } else {
//       // D k = mp.role ? 0.2 : 0.1225;
//       D k = 0.1225;
//       smin(maxSpeed, std::max(0.1, d) / k);
//       r.preferredVelocity = p * (maxSpeed / p.len());
//     }

//     if (r.isAttacker()) {
//       dbg(r.p, r.nextTo, r.preferredVelocity);
//     }
//   }

//   for (auto &r : robots) {
//     // auto os = Obstacle::getNearbyObstacles(mp.g, r.p, 5);
//     // HalfplaneBuilder hp(r, robots, os, r.radius + SELF_EXR, TIME_HORIZON,
//     //                     TIME_HORIZON_OBSTACLE);
//     // Pt optV = hp.solve(
//     //     std::fmin(MAX_FORWARD_SPEED[Map::instance().role],
//     //               r.v.len() + MAX_DELTA_LINE_SPEED_PER_FRAME[r.item > 0]));
//     // std::tie(r.lineSpeed, r.angleSpeed) = getArgs(r.angle, optV);
//     auto os = Obstacle::getNearbyObstacles(mp.g, r.p, 6);

//     D thObst = MAX_FORWARD_SPEED[mp.role] /
//                    MAX_DELTA_LINE_SPEED_PER_FRAME[r.item > 0] / FPS * 0.5 +
//                0.1;

//     HalfplaneBuilder hp(r, robots, os, r.radius + SELF_EXR, TIME_HORIZON,
//                         thObst);
//     Pt optV = hp.solve(
//         std::fmin(MAX_FORWARD_SPEED[mp.role],
//                   r.v.len() + MAX_DELTA_LINE_SPEED_PER_FRAME[r.item > 0]));

//     std::tie(r.lineSpeed, r.angleSpeed) =
//         getArgs(r, optV, hp.robotCollision, hp.obstCollision);

//     // if (mp.map_id != 1) {
//     // D disToRob = 1e9;
//     // D mneV = eV;
//     // for (auto &rb : robots)
//     //   if (rb.id != r.id) {
//     //     D angle_dis = normAngle(r.angle - r.p.angleTo(rb.p));
//     //     if (abs(angle_dis) <= PI / 3)
//     //       if (r.p.dis(rb.p) - r.radius - rb.radius < disToRob) {
//     //         disToRob = r.p.dis(rb.p) - r.radius - rb.radius;
//     //         mneV = rb.lineSpeed;
//     //       }
//     //   }
//     // if (disToRob <= 0.8)
//     //   smin(eV, mneV);
//     // }
//     // D disToRob = 1e9;
//     // for (auto &rb : robots)
//     //   if (rb.id != r.id) {
//     //     D angle_dis = normAngle(r.angle - r.p.angleTo(rb.p));
//     //     if (abs(angle_dis) <= PI / 2)
//     //       smin(disToRob, r.p.dis(rb.p) - r.radius - rb.radius - 0.4);
//     //   }

//     // if (disToRob <= 0.4)
//     //   smin(eV, disToRob);
//   }

// #ifdef DEMO_MOVE
//   for (auto &r : robots)
//     demoMove(r);
//   for (auto &r1 : robots) {
//     for (auto &r2 : robots) {
//       if (r1.id == r2.id)
//         continue;
//       if (!r1.state || !r2.state)
//         continue;
//       auto dis = r1.p.dis(r2.p);
//       if (dis <= r1.radius + r2.radius + 0.002) {
//         if (r1.p.x < r2.p.x) {
//           if (r1.angle > 0)
//             r1.angleSpeed += MAX_ROTATE_SPEED / 4.5;
//           else
//             r1.angleSpeed += -MAX_ROTATE_SPEED / 4.5;
//         } else {
//           if (r1.angle > 0)
//             r1.angleSpeed += -MAX_ROTATE_SPEED / 4.5;
//           else
//             r1.angleSpeed += MAX_ROTATE_SPEED / 4.5;
//         }
//       }
//     }
//   }
// #endif
//   for (Robot &rob : robots) {
//     std::cout << "forward " << rob.id << ' ' << rob.lineSpeed << '\n';
//     std::cout << "rotate " << rob.id << ' ' << rob.angleSpeed << '\n';
//   }
// }

inline void moveRobots()
{
  auto &mp = Map::instance();
  auto &mpFoe = Map::instanceFoe();
  auto &robots = mp.robots;

  static std::vector<int> ids;
  static std::vector<int> hPFrame;

  if (ids.empty())
  {
    ids.resize(robots.size());
    iota(ids.begin(), ids.end(), 0);
  }
  if (hPFrame.empty())
    hPFrame.resize(robots.size());

  stable_sort(ids.begin(), ids.end(), [&](int x, int y)
              {
    if (robots[x].item != robots[y].item)
      return robots[x].item > robots[y].item;
    return x < y; });

  // for (size_t i = 0; i < ids.size(); ++i)
  // {
  //   --hPFrame[ids[i]];
  //   bool canMove = false;
  //   auto &r = robots[ids[i]];
  //   // // 矩形写法
  //   // double mnDis = 1.4; // 参数，预测机器人能否往某个方向走mnDis米
  //   // for (double dir = -PI; dir <= PI; dir += PI / 30)
  //   // {
  //   //   bool flag = true;
  //   //   Point direction(cos(dir), sin(dir));
  //   //   Point nex = r.p + direction * mnDis;
  //   //   double robR = r.radius - 0.01; // 参数，影响矩形判断精度
  //   //   Rectangle rec = {r.p + direction.rotate(PI / 2) * robR,
  //   //                    r.p + direction.rotate(-PI / 2) * robR,
  //   //                    nex + direction.rotate(-PI / 2) * robR,
  //   //                    nex + direction.rotate(PI / 2) * robR};
  //   //   Circle cir = {nex, r.radius};
  //   //   double angleDis = normAngle(r.p.angleTo(nex) - r.angle);
  //   //   if (angleDis < 0)
  //   //     angleDis += 2 * PI;
  //   //   int mid = angleDis / PI * 180;
  //   //   int range = 89; // 参数，枚举雷达的范围
  //   //   for (int j = mid - range; j <= mid + range; ++j)
  //   //   {
  //   //     double len = r.radar[(j % 360 + 360) % 360];
  //   //     assert(len >= r.radius);
  //   //     double tmpAngle = r.angle + 1.0 * j / 180 * PI;
  //   //     Point obs = r.p + Point(cos(tmpAngle), sin(tmpAngle)) * len;
  //   //     if (rec.isIn(obs) || cir.isIn(obs))
  //   //     {
  //   //       // dbg(r.p.str(), nex.str(), mnDis, direction.str());
  //   //       // dbg(rec.p1.str(), rec.p2.str(), rec.p3.str(), rec.p4.str(),
  //   //       // obs.str()); exit(0);
  //   //       flag = false;
  //   //       break;
  //   //     }
  //   //   }
  //   //   if (flag)
  //   //   {
  //   //     canMove = true;
  //   //     break;
  //   //   }
  //   // }
  //   // //  矩形写法结束

  //   // // bfs写法
  //   // int gridStep = 4; //
  //   参数，机器人在格子地图能走至少gridStep步，则没被其它机器人夹住
  //   // auto obs =
  //   //     Obstacle::getNearbyObstaclePointsToSegment(mp.g, r.p, gridStep +
  //   2);
  //   // auto check = [&](int xx, int yy)
  //   // {
  //   //   const double checkEps = 0.02; // 参数，check精度
  //   //   if (xx <= 0 || xx >= mp.gn - 1 || yy <= 0 || yy >= mp.gn - 1)
  //   //     return false;
  //   //   Point p(xx * mp.gstep, yy * mp.gstep);
  //   //   for (auto &rb : robots)
  //   //     if (rb.id != r.id && rb.p.dis(p) < r.radius + rb.radius -
  //   checkEps)
  //   //       return false;
  //   //   for (auto &ob : obs)
  //   //     if (distPS(p, ob.first, ob.second) < r.radius - checkEps)
  //   //       return false;
  //   //   return true;
  //   // };

  //   // bool checkOk = false;
  //   // int x = r.p.x / mp.gstep, y = r.p.y / mp.gstep;
  //   // for (int dx = 0; dx <= 1; ++dx)
  //   //   for (int dy = 0; dy <= 1; ++dy)
  //   //     if (check(x + dx, y + dy))
  //   //     {
  //   //       x += dx;
  //   //       y += dy;
  //   //       checkOk = true;
  //   //       break;
  //   //     }
  //   // if (!checkOk)
  //   //   canMove = false;
  //   // else
  //   // {
  //   //   std::vector<std::vector<int>> dis(
  //   //       2 * gridStep + 1, std::vector<int>(2 * gridStep + 1,
  //   INT32_MAX));
  //   //   dis[gridStep][gridStep] = 0;
  //   //   std::queue<std::pair<int, int>> q;
  //   //   q.push({x, y});
  //   //   while (!q.empty())
  //   //   {
  //   //     int xx = q.front().first, yy = q.front().second;
  //   //     q.pop();
  //   //     Point p(xx * mp.gstep, yy * mp.gstep);
  //   //     if (!check(xx, yy))
  //   //       continue;
  //   //     if (dis[xx - x + gridStep][yy - y + gridStep] >= gridStep)
  //   //     {
  //   //       canMove = true;
  //   //       break;
  //   //     }
  //   //     for (int dx = -1; dx <= 1; ++dx)
  //   //       for (int dy = -1; dy <= 1; ++dy)
  //   //         if (dx || dy)
  //   //         {
  //   //           int xxx = xx + dx;
  //   //           int yyy = yy + dy;
  //   //           if (xxx >= x - gridStep && xxx <= x + gridStep &&
  //   //               yyy >= y - gridStep && yyy <= y + gridStep &&
  //   //               dis[xxx - x + gridStep][yyy - y + gridStep] >
  //   //                   dis[xx - x + gridStep][yy - y + gridStep] + 1)
  //   //           {
  //   //             dis[xxx - x + gridStep][yyy - y + gridStep] =
  //   //                 dis[xx - x + gridStep][yy - y + gridStep] + 1;
  //   //             q.push({xxx, yyy});
  //   //           }
  //   //         }
  //   //   }
  //   // }
  //   // // bfs写法结束

  //   // dbg(ids[i], canMove);
  //   const int C = 50; // 参数，保持最高有限制至少C帧
  //   if (!canMove)
  //     hPFrame[ids[i]] = C;
  //   if (hPFrame[ids[i]] > 0)
  //   {
  //     int j = i;
  //     while (j)
  //     {
  //       std::swap(ids[j - 1], ids[j]);
  //       --j;
  //     }
  //   }
  // }

  std::vector<std::pair<Pt, Pt>> segs;
  for (int id : ids)
  {
    auto &r = robots[id];
    if (r.isTransporter())
    {
      int pid = r.getTarget();
      r.nextTo = mp.getNextTo(r.p, pid, r.state == 1 ? r.buyPlatArea : 0,
                              r.radius, segs);

      Pt u = r.p, v = r.nextTo - u;
      if (v.len() < 0.001)
      {
        segs.push_back({u, u});
      }
      else
      {
        segs.push_back(
            {u - v.unit() * r.radius,
             u + v.unit() *
                     (r.radius +
                      std::min(v.len(),
                               (r.v.len() +
                                MAX_DELTA_LINE_SPEED_PER_FRAME[r.item > 0]) /
                                   FPS))});
      }
    }
  }
  for (int id : ids)
  {
    auto &r = robots[id];
    if (r.isAttacker())
    {
      if (r.attackBuy123 >= 0)
      {
        r.nextTo = mp.getNextTo(r.p, r.attackBuy123, 0, r.radius, segs);
        continue;
      }
      // dbg(r.attackFoeId, r.attackPlatId);
      if (r.attackFoeId !=
          -1)
      { // 雷达能照射到敌方机器人，则不太容易挡自己机器人的路
        auto &e = mp.foes[r.attackFoeId];
        r.nextTo = e.pos + e.velocity / FPS * 0; // 参数，预估帧数
      }
      else if (r.attackPlatId != -1)
        r.nextTo = mpFoe.getNextTo(r.p, r.attackPlatId, 0, r.radius, segs);
    }
  }

  for (Platform &p : mp.plats)
    p.freshVisit();
  for (Robot &rob : robots)
  {
    rob.fresh();
    rob.wait = false;
    int target = rob.getTarget();
    if (target != -2 && mp.getEvalDis(rob) <= 6)
      mp.plats[target].applyVisit(rob.id);
  }
  for (Platform &p : mp.plats)
    p.processApplyVisitList();

  for (Robot &rob : robots)
  {
    int target = rob.getTarget();
    if (target != -2 && mp.getEvalDis(rob) <= 6 &&
        mp.plats[target].visitRobotId != rob.id)
      rob.wait = true;
  }

  for (auto &r : robots)
  {
    int target = r.getTarget();
    D maxSpeed = MAX_FORWARD_SPEED[mp.role];
    if (r.wait)
      maxSpeed *= 0.05;
    Point p = r.nextTo - r.p;
    D d = p.len();
    if (target >= 0)
      d = r.p.dis(mp.plats[target].p);
    if (p.len() <= 1e-8)
    {
      r.preferredVelocity = Point(0, 0);
    }
    else
    {
      D k = mp.role ? 0.2222 : 0.1225;
      r.preferredVelocity =
          p / p.len() * std::min(maxSpeed, std::max(0.1, d) / k);
    }
  }

  for (auto &r : robots)
  {
    // auto os = Obstacle::getNearbyObstacles(mp.g, r.p, 5);
    // HalfplaneBuilder hp(r, robots, os, r.radius + SELF_EXR, TIME_HORIZON,
    //                     TIME_HORIZON_OBSTACLE);
    // Pt optV = hp.solve(
    //     std::fmin(MAX_FORWARD_SPEED[Map::instance().role],
    //               r.v.len() + MAX_DELTA_LINE_SPEED_PER_FRAME[r.item > 0]));
    // std::tie(r.lineSpeed, r.angleSpeed) = getArgs(r.angle, optV);

    auto os = Obstacle::getNearbyObstacles(mp.g, r.p, 6);

    D thObst = MAX_FORWARD_SPEED[mp.role] /
                   MAX_DELTA_LINE_SPEED_PER_FRAME[r.item > 0] / FPS * 0.5 +
               0.1;

    HalfplaneBuilder hp(r, robots, os, r.radius + SELF_EXR, TIME_HORIZON,
                        thObst);
    Pt optV = hp.solve(
        std::fmin(MAX_FORWARD_SPEED[mp.role],
                  r.v.len() + MAX_DELTA_LINE_SPEED_PER_FRAME[r.item > 0]));

    std::tie(r.lineSpeed, r.angleSpeed) =
        getArgs(r, optV, hp.robotCollision, hp.obstCollision);

    // if (mp.map_id != 1) {
    // D disToRob = 1e9;
    // D mneV = eV;
    // for (auto &rb : robots)
    //   if (rb.id != r.id) {
    //     D angle_dis = normAngle(r.angle - r.p.angleTo(rb.p));
    //     if (abs(angle_dis) <= PI / 3)
    //       if (r.p.dis(rb.p) - r.radius - rb.radius < disToRob) {
    //         disToRob = r.p.dis(rb.p) - r.radius - rb.radius;
    //         mneV = rb.lineSpeed;
    //       }
    //   }
    // if (disToRob <= 0.8)
    //   smin(eV, mneV);
    // }
    // D disToRob = 1e9;
    // for (auto &rb : robots)
    //   if (rb.id != r.id) {
    //     D angle_dis = normAngle(r.angle - r.p.angleTo(rb.p));
    //     if (abs(angle_dis) <= PI / 2)
    //       smin(disToRob, r.p.dis(rb.p) - r.radius - rb.radius - 0.4);
    //   }

    // if (disToRob <= 0.4)
    //   smin(eV, disToRob);
  }

// #define DEMO_MOVE
#ifdef DEMO_MOVE
  for (auto &r : robots)
    demoMove(r);
  for (auto &r1 : robots)
  {
    for (auto &r2 : robots)
    {
      if (r1.id == r2.id)
        continue;
      if (!r1.state || !r2.state)
        continue;
      auto dis = r1.p.dis(r2.p);
      if (dis <= r1.radius + r2.radius + 0.002)
      {
        if (r1.p.x < r2.p.x)
        {
          if (r1.angle > 0)
            r1.angleSpeed += MAX_ROTATE_SPEED / 4.5;
          else
            r1.angleSpeed += -MAX_ROTATE_SPEED / 4.5;
        }
        else
        {
          if (r1.angle > 0)
            r1.angleSpeed += -MAX_ROTATE_SPEED / 4.5;
          else
            r1.angleSpeed += MAX_ROTATE_SPEED / 4.5;
        }
      }
    }
  }
#endif
  for (Robot &rob : robots)
  {

    rob.lineSpeed = max(rob.lineSpeed, 0.1);

    if (rob.isTransporter())
    {
      if (rob.rushBackward != -1)
      {
        assert(rob.state == 2 && rob.item && rob.sell != -1);
        auto &p = mp.plats[rob.sell];
        double angleDis = normAngle(rob.angle - rob.p.angleTo(p.p));
        rob.angleSpeed = (angleDis < 0 ? -1 : 1) * MAX_ROTATE_SPEED * sin(abs(angleDis));
        if (abs(angleDis) > 0.08)
          rob.lineSpeed = 0;
        else
        {
          if (rob.rotateBackward == 0)
          {
            rob.lineSpeed = -MAX_BACKWARD_SPEED;
          }
          else
          {
            rob.lineSpeed = MAX_FORWARD_SPEED[mp.role];
          }
        }
      }
      else if (rob.rotateBackward > 0)
      {
        --rob.rotateBackward;
        rob.lineSpeed = -MAX_BACKWARD_SPEED;
        rob.angleSpeed = PI;
      }
    }

    std::cout << "forward " << rob.id << ' ' << rob.lineSpeed << '\n';
    std::cout << "rotate " << rob.id << ' ' << rob.angleSpeed << '\n';
  }
}