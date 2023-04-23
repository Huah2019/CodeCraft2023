#pragma once

#include "geo.hpp"

constexpr int MAP_SIZE = 100;
constexpr int GN = MAP_SIZE * 2;
constexpr D GSTEP = 50.0 / GN;
constexpr int FPS = 50;

#ifdef SEMI_VERSION
constexpr int FRAME_LIMIT = FPS * 60 * 5 - 100;
#else
constexpr int FRAME_LIMIT = FPS * 60 * 4 - 100;
#endif

constexpr D DIST_RW = 0.4;                  // 机器人-工作台判定距离
constexpr D R[2] = {0.45, 0.53};            // 机器人半径
constexpr D RHO = 20;                       // 机器人密度
constexpr int MAX_FORWARD_SPEED[] = {6, 7}; // 最大前进速度
constexpr int MAX_BACKWARD_SPEED = 2;       // 最大后退速度
constexpr D MAX_ROTATE_SPEED = PI;          // 最大旋转速度
constexpr D MAX_F = 250;                    // 最大牵引力
constexpr D MAX_M = 50;                     // 最大力矩

constexpr D TIME_STEP = D(1) / FPS;          // 每帧时间
constexpr D TIME_HORIZON = 17.5 * TIME_STEP; // ORCA 机器人预测时间
constexpr D TIME_HORIZON_OBSTACLE = 12.5 * TIME_STEP; // ORCA 障碍预测时间
constexpr D SELF_EXR = 0.02;
constexpr D OTHER_EXR = 0.05;
constexpr D ROBOT_NEIGHBOR_HORIZON = 5.0;
constexpr D FOE_NEIGHBOR_HORIZON = 20.0;
constexpr D OBST_NEIGHBOR_HORIZON = 10.0;

constexpr int GET_NEXT_TO_NEIGHBOR_RANGE = 2;

constexpr D MAX_DELTA_LINE_SPEED_PER_FRAME[2] = { // 每帧最大线速度变化量
    MAX_F / (R[0] * R[0] * RHO * PI) / FPS,
    MAX_F / (R[1] * R[1] * RHO * PI) / FPS};
// constexpr D MAX_DELTA_ROTATE_SPEED_PER_FRAME[2] = { // 每帧最大转速变化量
//     MAX_M / (R[0] * R[0] * R[0] * R[0] * RHO * PI * 25),
//     MAX_M / (R[1] * R[1] * R[1] * R[1] * RHO * PI * 25)};

const int BUY_PRICE[] = {0, 3000, 4400, 5800, 15400, 17200, 19200, 76000};
const int SELL_PRICE[] = {0, 6000, 7600, 9200, 22500, 25000, 27500, 105000};

inline D calAngCost(D ang, int role) {
  D angCost = ang / MAX_ROTATE_SPEED * MAX_FORWARD_SPEED[role];
  if (ang < PI / 6) {
    return angCost * 0.1;
  } else if (ang < PI / 4.6) {
    return angCost * 0.7;
  } else if (ang < PI / 4) {
    return angCost * 2 + 0.2;
  } else {
    return angCost * 2.8 + 0.5;
  }
}