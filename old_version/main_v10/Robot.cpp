#include "Robot.hpp"

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