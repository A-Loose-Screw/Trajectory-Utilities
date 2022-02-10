#ifndef CONTROL_THEORY_H
#define CONTROL_THEORY_H

#include "PID.h"
// #include "Trajectory/Trajectory.h"

/**
 * @brief Abstracted theory for movement
 * The abstracted/simplified goal of movement is to get from A to B,
 * where that point of A and B has been processed via some other means (trajectories).
 * 
 * Plan the control of movement getting from A through to B
 */
class ControlTheory {
 public:
  ControlTheory() {}

  void start() {}
  void stop() {}

  void update() {}
 private:
};

#endif