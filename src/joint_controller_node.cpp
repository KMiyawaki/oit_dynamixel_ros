#include <string>
#include <angles/angles.h>
#include <boost/bind.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include "ros/ros.h"
#include "oit_dynamixel_ros/joint_controller.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "joint_controller");
  JointController node;
  std::cout << node.toString() << std::endl;
  node.run();
  return 0;
}
