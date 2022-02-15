#include <iostream>
#include "TrajectoryUtilities.h"

Trajectory<Splines::CatmullRom> trajectory;

int main() {
  trajectory.push_back({
    {0,0}, {1,0}, {2,0}, {3,0}
  });
  

  trajectory.build();
  trajectory.simulate();

  // trajectory.print();

  return 0;
}