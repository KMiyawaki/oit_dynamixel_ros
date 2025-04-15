#include <string>
#include <angles/angles.h>
#include <boost/bind.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include "ros/ros.h"
#include "oit_dynamixel_ros/dynamixel_bus.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "dynamixel_bus");
  DynamixelBus node;
  node.run();
  ROS_INFO_STREAM(std::endl
                  << node.toString());
  ros::spin(); // Single thread
  return 0;
}
