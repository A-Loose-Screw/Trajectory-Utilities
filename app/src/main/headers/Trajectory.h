#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "Splines/CatmullRom.h"

template<typename SplineType>
class Trajectory {
 public:
  Trajectory() {}
  Trajectory(std::vector<Splines::Waypoint> waypoints) {
    _trajectory.push_back({waypoints});
  }

  void push_back(std::vector<Splines::Waypoint> waypoints) {
    _trajectory.push_back({waypoints});
  }

  void pop_back() {
    _trajectory.pop_back();
  }

  void build() {
    for (auto &spline : _trajectory) {
      if (_sType.calculateSpline(spline) == -1) {
        std::cout << "Error while calculating spline in trajectory" << std::endl;
      }
    }
  }

  Splines::Spline getSpline(int spline) {
    return _trajectory[spline];
  }

  std::vector<Splines::Spline> &getRawTrajectory() { 
    return _trajectory; 
  }

  friend std::ostream &operator<<(std::ostream &os, const Trajectory &traj) {
    os << "-- Trajectory --\n";
    os << "| Type: " << typeid(SplineType).name() << "\n";
    return os;
  }

  SplineType getType() {
    return _sType;
  }

 private:
  std::vector<Splines::Spline> _trajectory;
  SplineType _sType;
};

#endif