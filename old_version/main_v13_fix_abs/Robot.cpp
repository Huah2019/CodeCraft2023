#include "Robot.hpp"
using namespace std;

int Robot::getTarget()
{
  if (state == 0)
    return -2;
  if (state == 1)
    return buy;
  return sell;
}
void Robot::fresh() { radius = R[item > 0]; }

void Robot::readState()
{
  std::cin >> pid >> item >> tf >> cf;
  std::cin >> angleSpeed >> v.x >> v.y >> angle;
  std::cin >> p.x >> p.y;
  lineSpeed = v.len();
}
void Robot::readRadar()
{
#ifndef SEMI_VERSION
  for (int i = 0; i < 360; ++i)
    std::cin >> radar[i];
#endif
}
void Robot::updIsStop()
{
  int K = 80;
  int T = 50;
  double mnV = 0.05;
  int sum = 0;
  for (int i = max(0, int(hisVelocity.size()) - 1); i < int(hisVelocity.size()); ++i)
    sum += hisVelocity[i] <= mnV;
  isStop = sum >= T;
}
bool Robot::checkIsStop()
{
  int K = 80;
  int T = 50;
  double mnV = 0.05;
  int sum = 0;
  for (int i = max(0, int(hisVelocity.size()) - K); i < int(hisVelocity.size()); ++i)
    sum += hisVelocity[i] <= mnV;
  return sum >= T;
}