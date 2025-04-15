#ifndef ___DYNAMIXEL_BUS_H___
#define ___DYNAMIXEL_BUS_H___
#include <string>
#include <deque>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <oit_dynamixel_ros/ServoCommand.h>
#include "dx2lib.h"

class DynamixelBus
{
public:
  DynamixelBus();
  ~DynamixelBus();

  std::string toString() const;
  bool run();
  void stop();
  bool isOpened() const;
  bool sendCommand(const oit_dynamixel_ros::ServoCommand &command);
  inline const std::vector<uint8_t> &getServoIDs() const
  {
    return this->ids_;
  }
  inline size_t getServoSize() const
  {
    return this->getServoIDs().size();
  }
  bool setTorqueEnable(bool enable);
  bool hasId(uint8_t id) const;

private:
  TDeviceID device_;
  std::string port_;
  int baud_rate_;
  std::vector<uint8_t> ids_;

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber command_sub_;
  ros::Publisher state_pub_;
  int state_seq_;
  ros::Timer timer_;

  ros::ServiceServer dump_srv_;

  void commandCallback(const oit_dynamixel_ros::ServoCommandConstPtr &msg);
  void timerCallback(const ros::TimerEvent &event);
  bool dumpSrvCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
};

#endif
