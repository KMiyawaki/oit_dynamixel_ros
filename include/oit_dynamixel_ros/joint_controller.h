#ifndef ___DYNAMIXEL_CONTROLLER_H___
#define ___DYNAMIXEL_CONTROLLER_H___

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include "oit_dynamixel_ros/utils.h"
#include "oit_dynamixel_ros/joint_info.h"
#include "oit_dynamixel_ros/ServoCommand.h"
#include "oit_dynamixel_ros/ServoState.h"
#include "dx2lib.h"

class JointController
{
public:
  explicit JointController();
  virtual ~JointController();

  std::string toString() const;
  void run();
  JointInfo::ConstPtr getByName(const std::string &name) const;
  JointInfo::ConstPtr getById(int id) const;
  inline size_t getJointSize() const
  {
    return this->joints_.size();
  }
  inline bool jointNamesToServoIds(const std::vector<std::string> &joint_names, std::vector<uint8_t> &ids) const
  {
    control_msgs::FollowJointTrajectoryResult result;
    return this->jointNamesToServoIds(joint_names, ids, result);
  }
  inline bool validateJointNames(const std::vector<std::string> &joint_names) const
  {
    control_msgs::FollowJointTrajectoryResult result;
    return this->validateJointNames(joint_names, result);
  }
  inline bool validateJointTrajectory(const trajectory_msgs::JointTrajectory &traj) const
  {
    control_msgs::FollowJointTrajectoryResult result;
    return this->validateJointTrajectory(traj, result);
  }

private:
  typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;
  typedef boost::shared_ptr<Server> ServerPtr;
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;
  typedef boost::shared_ptr<Client> ClientPtr;

  JointInfo::Container joints_;
  ros::NodeHandle nh_, pnh_;
  ros::AsyncSpinner spinner_;

  // Recv servo states, publish joint states
  ros::Subscriber servo_state_sub_;
  ros::Publisher joint_state_pub_;
  int joint_state_seq_;
  sensor_msgs::JointStatePtr joint_state_;
  boost::mutex joint_state_muntex_;

  // Recv joint trajectory, resend as action.
  ros::Subscriber joint_trajectory_sub_;

  // Action server and client
  // Thread 3-5
  ServerPtr action_server_;
  ClientPtr action_client_;
  double process_rate_;
  int action_feedback_seq_;
  int servo_command_seq_;
  ros::Publisher servo_command_pub_;

  // Thread 1
  void servoStateCallback(const oit_dynamixel_ros::ServoStateConstPtr &msg);
  void servoStateToJointState(const oit_dynamixel_ros::ServoState &state, sensor_msgs::JointState& joint_state);
  // Thread 2
  void jointTrajectoryCallback(const trajectory_msgs::JointTrajectoryConstPtr &msg);

  bool loadJoint(const std::string &name, const XmlRpc::XmlRpcValue &val);
  template <class TAG, class VAL>
  JointInfo::ConstPtr getBy(VAL v) const
  {
    typedef typename JointInfo::Container::index<TAG>::type MAP;
    const MAP &mp = this->joints_.get<TAG>();
    typename MAP::const_iterator cit = mp.find(v);
    if (cit == mp.end())
    {
      return JointInfo::ConstPtr();
    }
    return *cit;
  }
  bool validateJointTrajectory(const trajectory_msgs::JointTrajectory &traj, control_msgs::FollowJointTrajectoryResult &result) const;
  bool jointNamesToServoIds(const std::vector<std::string> &joint_names, std::vector<uint8_t> &ids, control_msgs::FollowJointTrajectoryResult &result) const;
  inline bool validateJointNames(const std::vector<std::string> &joint_names, control_msgs::FollowJointTrajectoryResult &result) const
  {
    std::vector<uint8_t> ids;
    return this->jointNamesToServoIds(joint_names, ids, result);
  }
  bool getCommand(const trajectory_msgs::JointTrajectory &traj, const std::vector<uint8_t> &ids, size_t index, double duration_offset, oit_dynamixel_ros::ServoCommand &cmd, control_msgs::FollowJointTrajectoryResult &result);
  void executeCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);
  void buildFeedback(const std::vector<std::string>& joint_names, const trajectory_msgs::JointTrajectoryPoint& target, double elapsed, control_msgs::FollowJointTrajectoryFeedback& feedback);
};

#endif
