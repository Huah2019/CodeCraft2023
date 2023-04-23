#pragma once
#include "Geometry.hpp"

constexpr int MAP_SIZE = 100;
constexpr int FPS = 50;
constexpr int FRAME_LIMIT = FPS * 60 * 5; // 总时长 5 分钟

constexpr D DIST_RW = 0.4;            // 机器人-工作台判定距离
constexpr D R[2] = {0.45, 0.53};      // 机器人半径
constexpr D RHO = 20;                 // 机器人密度
constexpr int MAX_FORWARD_SPEED = 6;  // 最大前进速度
constexpr int MAX_BACKWARD_SPEED = 2; // 最大后退速度
constexpr D MAX_ROTATE_SPEED = PI;    // 最大旋转速度
constexpr D MAX_F = 250;              // 最大牵引力
constexpr D MAX_M = 50;               // 最大力矩

constexpr D TIME_STEP = D(1) / FPS * 0.5;           // 每帧时间
constexpr D TIME_HORIZON = 35 * TIME_STEP;          // ORCA 机器人预测时间
constexpr D TIME_HORIZON_OBSTACLE = 25 * TIME_STEP; // ORCA 障碍预测时间
// constexpr D ORCA_RANGE = R[1] * 5;                 // ORCA neighbor 距离

constexpr D MAX_DELTA_LINE_SPEED_PER_FRAME[2] = { // 每帧最大线速度变化量
    MAX_F / (R[0] * R[0] * RHO * PI) / 50,
    MAX_F / (R[1] * R[1] * RHO * PI) / 50};
constexpr D MAX_DELTA_ROTATE_SPEED_PER_FRAME[2] = { // 每帧最大转速变化量
    MAX_M / (R[0] * R[0] * R[0] * R[0] * RHO * PI * 25),
    MAX_M / (R[1] * R[1] * R[1] * R[1] * RHO * PI * 25)};

constexpr int DIVIDE_SIZE = 2;
// constexpr int DIVIDED_MAP_SIZE = MAP_SIZE * DIVIDE_SIZE;
// constexpr D DIVIDE_STEP = 50.0 / DIVIDED_MAP_SIZE;
