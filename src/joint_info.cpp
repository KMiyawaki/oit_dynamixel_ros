#include "oit_dynamixel_ros/joint_info.h"

std::string JointInfo::toString() const
{
  std::stringstream sst;
  sst << this->name << ": id = " << this->id;
  return sst.str();
}
