#include <algorithm>
#include <iomanip>
#include <boost/bind/bind.hpp>
#include "oit_dynamixel_ros/dynamixel_bus.h"
#include "oit_dynamixel_ros/ServoState.h"

DynamixelBus::DynamixelBus() : device_(0), pnh_("~")
{
}

DynamixelBus::~DynamixelBus()
{
  this->stop();
}

std::string DynamixelBus::toString() const
{
  std::stringstream sst;
  sst << "port: " << this->port_ << std::endl;
  sst << "baud: " << this->baud_rate_ << std::endl;
  sst << "opened: " << std::boolalpha << this->isOpened() << std::endl;
  sst << "servos: ";
  for (size_t i = 0, i_end = this->getServoSize(); i < i_end; ++i)
  {
    sst << (int)this->getServoIDs()[i] << " ";
  }
  return sst.str();
}

bool DynamixelBus::run()
{
  double process_rate = 10.0;
  this->stop();
  this->state_seq_ = 0;
  this->pnh_.getParam("port", this->port_);
  this->pnh_.getParam("baud_rate", this->baud_rate_);
  this->pnh_.getParam("process_rate", process_rate);
  std::vector<int> motor_id_range;
  this->pnh_.getParam("motor_id_range", motor_id_range);
  if (motor_id_range.size() < 2)
  {
    ROS_ERROR_STREAM(ros::this_node::getName() << ":motor_id_range.size() less than 2");
    return false;
  }
  char path[1024];
  strcpy(path, this->port_.c_str());
  this->device_ = DX2_OpenPort(path, this->baud_rate_);
  if (this->device_ == 0)
  {
    ROS_ERROR_STREAM(ros::this_node::getName() << ": Failed to open " << path);
    this->stop();
    return false;
  }
  else
  {
    ROS_INFO_STREAM(ros::this_node::getName() << ": " << path << " opened successfully");
  }
  for (int i = *std::min_element(motor_id_range.begin(), motor_id_range.end()), i_end = *std::max_element(motor_id_range.begin(), motor_id_range.end()); i <= i_end; ++i)
  {
    TErrorCode err;
    if (DX2_Ping(this->device_, i, &err))
    {
      PDXL_ModelInfo p = DXL_GetModelInfo(this->device_, i);
      ROS_INFO_STREAM(ros::this_node::getName() << ": Found ID " << i << " Model " << p->name << " MaxVelocity " << p->velocitylimit.max * p->velocityratio << " [deg/sec]");
      this->ids_.push_back(i);
    }
  }
  DXL_SetDriveModesEquival(this->device_, this->getServoIDs().data(), this->getServoSize(), 0x04);
  DXL_SetOperatingModesEquival(this->device_, this->getServoIDs().data(), this->getServoSize(), 3);
  this->setTorqueEnable(true);

  this->command_sub_ = this->pnh_.subscribe<oit_dynamixel_ros::ServoCommand>("servo_command", 10, boost::bind(&DynamixelBus::commandCallback, this, _1));
  this->state_pub_ = this->pnh_.advertise<oit_dynamixel_ros::ServoState>("servo_state", 1);
  this->timer_ = this->nh_.createTimer(ros::Duration(1.0 / process_rate), &DynamixelBus::timerCallback, this);
  this->dump_srv_ = this->pnh_.advertiseService("dump_service", &DynamixelBus::dumpSrvCallback, this);
  return true;
}

void DynamixelBus::stop()
{
  this->timer_.stop();
  if (this->isOpened())
  {
    DX2_ClosePort(this->device_);
    this->device_ = 0;
  }
  ids_.clear();
  this->command_sub_.shutdown();
  this->state_pub_.shutdown();
}

bool DynamixelBus::isOpened() const
{
  return this->device_ != 0;
}

bool DynamixelBus::sendCommand(const oit_dynamixel_ros::ServoCommand &command)
{
  size_t size = command.ids.size();
  for (size_t i = 0; i < size; ++i)
  {
    if (this->hasId(command.ids[i]) == false)
    {
      ROS_ERROR_STREAM(ros::this_node::getName() << ": from " << command.sender << " : ID " << command.ids[i] << " is not registered");
      return false;
    }
  }
  if (size != command.angles.size())
  {
    ROS_ERROR_STREAM(ros::this_node::getName() << ": from " << command.sender << " size != command.angles.size()");
    return false;
  }
  if (command.duration > 0)
  {
    return DXL_SetGoalAnglesAndTime2(this->device_, command.ids.data(), command.angles.data(), size, command.duration);
  }
  if (size != command.velocities.size())
  {
    ROS_ERROR_STREAM(ros::this_node::getName() << ": from " << command.sender << " size != command.velocities.size()");
    return false;
  }
  std::vector<TAngleVelocity> angles_velocities;
  for (size_t i = 0; i < size; ++i)
  {
    TAngleVelocity angle_velocity;
    angle_velocity.angle = command.angles[i];
    angle_velocity.velocity = command.velocities[i];
    angles_velocities.push_back(angle_velocity);
  }
  return DXL_SetGoalAnglesAndVelocities(this->device_, command.ids.data(), angles_velocities.data(), size);
}

bool DynamixelBus::setTorqueEnable(bool enable)
{
  if (this->isOpened() == false)
  {
    return false;
  }
  return DXL_SetTorqueEnablesEquival(this->device_, this->getServoIDs().data(), this->getServoSize(), enable);
}

bool DynamixelBus::hasId(uint8_t id) const
{
  std::vector<uint8_t>::const_iterator it = std::find(this->ids_.cbegin(), this->ids_.cend(), id);
  return it != this->ids_.cend();
}

void DynamixelBus::timerCallback(const ros::TimerEvent &event)
{
  if (this->isOpened() == false)
  {
    return;
  }
  if (event.last_real.isValid() == false) // Skip first frame
  {
    return;
  }
  oit_dynamixel_ros::ServoState state;
  state.header.frame_id = "";
  state.header.seq = this->state_seq_++;
  state.header.stamp = event.current_real;
  state.ids = this->getServoIDs();
  size_t size = this->getServoSize();
  state.angles.assign(size, -1);
  state.velocities.assign(size, -1);
  state.currents.assign(size, -1);
  if (DXL_GetPresentAngles(this->device_, this->getServoIDs().data(), state.angles.data(), size) == false)
  {
    ROS_ERROR_STREAM(ros::this_node::getName() << ": DXL_GetPresentAngles failed");
    return;
  }
  if (DXL_GetPresentVelocities(this->device_, this->getServoIDs().data(), state.velocities.data(), size) == false)
  {
    ROS_ERROR_STREAM(ros::this_node::getName() << ": DXL_GetPresentVelocities failed");
    return;
  }
  if (DXL_GetPresentCurrents(this->device_, this->getServoIDs().data(), state.currents.data(), size) == false)
  {
    ROS_ERROR_STREAM(ros::this_node::getName() << ": DXL_GetPresentCurrents failed");
    return;
  }
  for (size_t i = 0; i < size; ++i)
  {
    state.currents[i] = state.currents[i] / 1000.0;
  }
  this->state_pub_.publish(state);
}
// private
void DynamixelBus::commandCallback(const oit_dynamixel_ros::ServoCommandConstPtr &msg)
{
  ROS_DEBUG_STREAM(ros::this_node::getName() << ": Recv command");
  this->sendCommand(*msg);
}

bool DynamixelBus::dumpSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::stringstream sst;
  sst << ros::this_node::getName() << " dump service" << std::endl;
  sst << "port: " << this->port_ << std::endl;
  sst << "baud: " << this->baud_rate_ << std::endl;
  sst << "open: " << std::boolalpha << this->isOpened() << std::endl;
  for (size_t i = 0; i < this->getServoSize(); ++i)
  {
    int id = this->getServoIDs()[i];
    PDXL_ModelInfo p = DXL_GetModelInfo(this->device_, id);
    sst << "ID: " << id << " Model " << p->name << " MaxVelocity " << p->velocitylimit.max * p->velocityratio << " [deg/sec]" << std::endl;
  }
  ROS_INFO_STREAM(std::endl
                  << sst.str());
  return true;
}
