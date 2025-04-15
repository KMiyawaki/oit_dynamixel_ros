#ifndef ___UTILS_H___
#define ___UTILS_H___

#include <trajectory_msgs/JointTrajectory.h>

void pushTrajectoryPoint(trajectory_msgs::JointTrajectory &traj, double duration, const std::vector<double> &positions);

class FixedTimer
{
public:
  explicit FixedTimer(double duration = 0) : duration_(duration), elapsed_(0) {}
  virtual ~FixedTimer() {}
  bool update(double delta);
  bool update(double delta, double &elapsed);
  inline void set(double duration)
  {
    this->duration_ = duration;
    this->reset();
  }
  inline void reset()
  {
    this->elapsed_ = 0;
  }

private:
  double duration_;
  double elapsed_;
};

#endif
