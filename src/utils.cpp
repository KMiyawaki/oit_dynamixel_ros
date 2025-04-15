#include "oit_dynamixel_ros/utils.h"

void pushTrajectoryPoint(trajectory_msgs::JointTrajectory &traj, double duration, const std::vector<double> &positions)
{
  trajectory_msgs::JointTrajectoryPoint pt;
  if (traj.points.empty())
  {
    pt.time_from_start = ros::Duration(duration);
  }
  else
  {
    pt.time_from_start = traj.points.rbegin()->time_from_start + ros::Duration(duration);
  }
  pt.positions = positions;
  traj.points.push_back(pt);
}

bool FixedTimer::update(double delta)
{
  double elapsed;
  return this->update(delta, elapsed);
}

bool FixedTimer::update(double delta, double &elapsed)
{
  bool result = false;
  this->elapsed_ += delta;
  if (this->elapsed_ > this->duration_)
  {
    this->elapsed_ = 0;
    result = true;
  }
  elapsed = this->elapsed_;
  return result;
}
