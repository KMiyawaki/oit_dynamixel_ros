#include <sstream>
#include <angles/angles.h>
#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include "dx2lib.h"
#include "oit_dynamixel_ros/joint_controller.h"
#include "oit_dynamixel_ros/ServoCommand.h"
//
JointController::JointController() : pnh_("~"), nh_(), spinner_(5), servo_command_seq_(0), joint_state_seq_(0), action_feedback_seq_(0), joint_state_(new sensor_msgs::JointState())
{
}

JointController::~JointController()
{
  this->spinner_.stop();
}

void JointController::jointTrajectoryCallback(const trajectory_msgs::JointTrajectoryConstPtr &msg)
{
  if (this->validateJointTrajectory(*msg) == false)
  {
    return;
  }
  if (this->action_client_ == NULL)
  {
    ROS_ERROR_STREAM(ros::this_node::getName() << ": Action client is null");
    return;
  }
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.goal_time_tolerance = ros::Duration(1.0);
  goal.trajectory = *msg;
  this->action_client_->sendGoal(goal);
}

void JointController::servoStateToJointState(const oit_dynamixel_ros::ServoState &state, sensor_msgs::JointState &js)
{
  js.header.seq = this->joint_state_seq_++;
  js.header.stamp = ros::Time::now();
  js.header.frame_id = "";
  const size_t size = state.ids.size();
  js.effort.assign(size, -1);
  js.name.assign(size, "");
  js.position.assign(size, -1);
  js.velocity.assign(size, -1);
  for (size_t i = 0; i < size; ++i)
  {
    JointInfo::ConstPtr pJ = this->getById(state.ids[i]);
    if (pJ == NULL)
    {
      continue;
    }
    js.name[i] = pJ->name;
    js.position[i] = angles::from_degrees(state.angles[i]);
    js.velocity[i] = angles::from_degrees(state.velocities[i]);
    js.effort[i] = state.currents[i];
  }
}

void JointController::servoStateCallback(const oit_dynamixel_ros::ServoStateConstPtr &msg)
{
  boost::mutex::scoped_lock lock(this->joint_state_muntex_);
  this->servoStateToJointState(*msg, *this->joint_state_);
  this->joint_state_pub_.publish(*this->joint_state_);
}

void JointController::run()
{
  this->pnh_.getParam("process_rate", this->process_rate_);
  if (this->process_rate_ < 0)
  {
    ROS_WARN_STREAM(ros::this_node::getName() << ": process_rate < 0");
    this->process_rate_ = 100;
  }
  XmlRpc::XmlRpcValue params;
  this->pnh_.getParam(ros::this_node::getName(), params);
  for (XmlRpc::XmlRpcValue::const_iterator cit = params.begin(), cit_end = params.end(); cit != cit_end; ++cit)
  {
    if (cit->first == "process_rate")
    {
      continue;
    }
    if (this->loadJoint(cit->first, cit->second) == false)
    {
      ROS_ERROR_STREAM(ros::this_node::getName() << " :Failed to load " << cit->first.c_str());
    }
  }

  this->joint_state_pub_ = this->nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
  this->servo_state_sub_ = this->nh_.subscribe<oit_dynamixel_ros::ServoState>("dynamixel_bus/servo_state", 1, boost::bind(&JointController::servoStateCallback, this, _1));
  this->joint_trajectory_sub_ = this->pnh_.subscribe<trajectory_msgs::JointTrajectory>("joint_trajectory", 1, boost::bind(&JointController::jointTrajectoryCallback, this, _1));
  this->servo_command_pub_ = this->nh_.advertise<oit_dynamixel_ros::ServoCommand>("dynamixel_bus/servo_command", 1);

  const std::string action_name = "follow_joint_trajectory";
  const std::string action_name_full = this->pnh_.getNamespace() + "/" + action_name;
  this->action_server_.reset(new Server(this->pnh_, action_name, boost::bind(&JointController::executeCallback, this, _1), false));
  this->action_server_->start();

  this->action_client_.reset(new Client(action_name_full, true));
  this->spinner_.start();

  if (this->action_client_->waitForServer(ros::Duration(5)) == false)
  {
    ROS_ERROR_STREAM(ros::this_node::getName() << ": Failed to connect to action server");
    this->action_client_.reset();
  }
  else
  {
    ROS_INFO_STREAM(ros::this_node::getName() << ": Connected to action server" << action_name_full);
  }
  ros::waitForShutdown(); // メインスレッドはシャットダウンを待機
}

void JointController::executeCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
  control_msgs::FollowJointTrajectoryResult result;
  if (this->validateJointTrajectory(goal->trajectory, result) == false)
  {
    this->action_server_->setAborted(result);
    return;
  }
  std::vector<uint8_t> ids;
  int crnt_waypoint_index = 0;
  const size_t joint_size = goal->trajectory.joint_names.size();
  const size_t waypoint_size = goal->trajectory.points.size();
  double elapsed = 0;
  this->jointNamesToServoIds(goal->trajectory.joint_names, ids);
  oit_dynamixel_ros::ServoCommand cmd;
  if (this->getCommand(goal->trajectory, ids, crnt_waypoint_index, 0, cmd, result) == false)
  {
    this->action_server_->setAborted(result);
    return;
  }
  this->servo_command_pub_.publish(cmd);
  ros::Rate r(this->process_rate_);
  ros::Time tm_pre = ros::Time::now();
  while (ros::ok())
  {
    if (this->action_server_->isPreemptRequested())
    {
      this->action_server_->setPreempted(result);
      break;
    }
    ros::Time tm = ros::Time::now();
    double delta = (tm - tm_pre).toSec();
    tm_pre = tm;
    elapsed += delta;
    const trajectory_msgs::JointTrajectoryPoint &pt = goal->trajectory.points[crnt_waypoint_index];
    control_msgs::FollowJointTrajectoryFeedback feedback;
    this->buildFeedback(goal->trajectory.joint_names, pt, elapsed, feedback);
    this->action_server_->publishFeedback(feedback);
    if (elapsed < pt.time_from_start.toSec())
    {
      // Pass
    }
    else if (crnt_waypoint_index >= waypoint_size - 1)
    {
      std::stringstream sst;
      sst << ros::this_node::getName() << ": Finished to follow joint trajectory within " << elapsed << " sec";
      result.error_code = result.SUCCESSFUL;
      result.error_string = sst.str();
      ROS_INFO_STREAM(sst.str());
      // We should check angles and goals...
      this->action_server_->setSucceeded(result);
      break;
    }
    else
    {
      ROS_INFO_STREAM(ros::this_node::getName() << ": Finished waypoint " << crnt_waypoint_index << " within " << elapsed << " sec");
      double duration_offset = pt.time_from_start.toSec() - elapsed;
      crnt_waypoint_index++;
      if (this->getCommand(goal->trajectory, ids, crnt_waypoint_index, duration_offset, cmd, result) == false)
      {
        this->action_server_->setAborted(result);
        return;
      }
      this->servo_command_pub_.publish(cmd);
    }
    r.sleep();
  }
}

void JointController::buildFeedback(const std::vector<std::string> &joint_names, const trajectory_msgs::JointTrajectoryPoint &target, double elapsed, control_msgs::FollowJointTrajectoryFeedback &feedback)
{
  boost::mutex::scoped_lock lock(this->joint_state_muntex_);
  feedback.header.stamp = ros::Time::now();
  feedback.header.frame_id = "";
  feedback.header.seq = this->action_feedback_seq_++;
  feedback.joint_names = joint_names;
  feedback.desired = target;
  feedback.actual.time_from_start = ros::Duration(elapsed);
  feedback.actual.positions = this->joint_state_->position;
  feedback.actual.effort = this->joint_state_->effort;
  feedback.actual.velocities = this->joint_state_->velocity;
  feedback.actual.accelerations.assign(feedback.actual.positions.size(), -1);
  trajectory_msgs::JointTrajectoryPoint &act = feedback.actual;
  trajectory_msgs::JointTrajectoryPoint &des = feedback.desired;
  trajectory_msgs::JointTrajectoryPoint &err = feedback.error;
  const size_t joint_size = joint_names.size();
  for (size_t i = 0; i < joint_size; ++i)
  {
    err.time_from_start = des.time_from_start - act.time_from_start;
    if (des.positions.size() > i && act.positions.size() > i)
    {
      err.positions.push_back(des.positions[i] - act.positions[i]);
    }
    if (des.velocities.size() > i && act.velocities.size() > i)
    {
      err.velocities.push_back(des.velocities[i] - act.velocities[i]);
    }
    if (des.accelerations.size() > i && act.accelerations.size() > i)
    {
      err.accelerations.push_back(des.accelerations[i] - act.accelerations[i]);
    }
  }
}

std::string JointController::toString() const
{
  std::stringstream sst;
  typedef JointInfo::Container::index<JointInfo::tagVector>::type Vector;
  const Vector &v = this->joints_.get<JointInfo::tagVector>();
  for (size_t i = 0; i < v.size(); ++i)
  {
    sst << v[i]->toString() << std::endl;
  }
  return sst.str();
}

JointInfo::ConstPtr JointController::getByName(const std::string &name) const
{
  return getBy<JointInfo::tagName, const std::string &>(name);
}

JointInfo::ConstPtr JointController::getById(int id) const
{
  return getBy<JointInfo::tagId, int>(id);
}

bool JointController::getCommand(const trajectory_msgs::JointTrajectory &traj, const std::vector<uint8_t> &ids, size_t index, double duration_offset, oit_dynamixel_ros::ServoCommand &cmd, control_msgs::FollowJointTrajectoryResult &result)
{
  if (index >= traj.points.size())
  {
    std::stringstream sst;
    sst << ros::this_node::getName() << ": index[" << index << "] >= points.size() [" << traj.points.size() << "]";
    result.error_code = result.INVALID_GOAL;
    result.error_string = sst.str();
    ROS_ERROR_STREAM(sst.str());
    return false;
  }
  const trajectory_msgs::JointTrajectoryPoint &pt = traj.points[index];
  cmd.header.seq = this->servo_command_seq_++;
  cmd.header.stamp = ros::Time::now();
  if (index == 0)
  {
    cmd.duration = pt.time_from_start.toSec();
  }
  else
  {
    cmd.duration = (pt.time_from_start - traj.points[index - 1].time_from_start).toSec();
  }
  cmd.duration += duration_offset;
  cmd.ids = ids;
  cmd.angles.clear();
  cmd.velocities.clear();
  cmd.sender = ros::this_node::getName();

  const size_t size = ids.size();
  if (pt.positions.size() != size)
  {
    std::stringstream sst;
    sst << ros::this_node::getName() << ": points[" << index << "].positions.size() is != servo size";
    result.error_code = result.INVALID_GOAL;
    result.error_string = sst.str();
    ROS_ERROR_STREAM(sst.str());
    return false;
  }
  for (size_t i = 0; i < size; ++i)
  {
    cmd.angles.push_back(angles::to_degrees(pt.positions[i]));
    if (i < pt.velocities.size())
    {
      cmd.velocities.push_back(angles::to_degrees(pt.velocities[i]));
    }
  }
  return true;
}

bool JointController::jointNamesToServoIds(const std::vector<std::string> &joint_names, std::vector<uint8_t> &ids, control_msgs::FollowJointTrajectoryResult &result) const
{
  ids.clear();
  for (size_t i = 0, i_end = joint_names.size(); i < i_end; ++i)
  {
    JointInfo::ConstPtr pJ = this->getByName(joint_names[i]);
    if (pJ == NULL)
    {
      std::stringstream sst;
      sst << ros::this_node::getName() << ": Joint " << joint_names[i] << " is not found";
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      result.error_string = sst.str();
      ROS_ERROR_STREAM(sst.str());
      ids.clear();
      return false;
    }
    ids.push_back(pJ->id);
  }
  return true;
}
// private:
bool JointController::validateJointTrajectory(const trajectory_msgs::JointTrajectory &traj, control_msgs::FollowJointTrajectoryResult &result) const
{
  std::stringstream sst;
  if (traj.joint_names.empty())
  {
    sst << ros::this_node::getName() << ": Joint name is empty";
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
    result.error_string = sst.str();
    ROS_ERROR_STREAM(sst.str());
    return false;
  }
  if (this->validateJointNames(traj.joint_names, result) == false)
  {
    return false;
  }
  const size_t joint_size = traj.joint_names.size();
  for (size_t i = 0, i_end = traj.points.size(); i < i_end; ++i)
  {
    const trajectory_msgs::JointTrajectoryPoint &pt = traj.points[i];
    if (pt.positions.size() < joint_size)
    {
      sst << ros::this_node::getName() << ": points[" << i << "].positions.size() is less than joint_names.size()";
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
      result.error_string = sst.str();
      ROS_ERROR_STREAM(sst.str());
      return false;
    }
    if (pt.time_from_start.toSec() > 0)
    {
      continue;
    }
    else
    {
      sst << ros::this_node::getName() << ": points[" << i << "].positions.time_from_start.toSec() is less than 0";
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
      result.error_string = sst.str();
      ROS_ERROR_STREAM(sst.str());
      return false;
    }
  }
  return true;
}

bool JointController::loadJoint(const std::string &name, const XmlRpc::XmlRpcValue &val)
{
  int id = -1;
  if (val.hasMember("ID"))
  {
    id = (int)val["ID"];
  }
  else if (val.hasMember("id"))
  {
    id = (int)val["id"];
  }
  JointInfo::Ptr pJ(new JointInfo(id, name));
  if (val.hasMember("max_velocity"))
  {
    pJ->max_velocity = (double)val["max_velocity"];
  }
  else
  {
    ROS_WARN_STREAM(ros::this_node::getName() << ": ID[" << id << "] does not have max velocity");
  }
  joints_.insert(pJ);
  return true;
}
