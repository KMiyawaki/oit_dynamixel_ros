#!/usr/bin/env python
import math
import os

import actionlib
import rospy
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def make_joint_trajectory(seq, joint_names):
    jt = JointTrajectory()
    jt.header.stamp = rospy.Time.now()
    jt.header.frame_id = ''
    jt.header.seq = seq
    jt.joint_names = joint_names
    return jt


def push_trajectory_point(joint_trajectory, duration, positions):
    pt = JointTrajectoryPoint()
    if not joint_trajectory.points:
        pt.time_from_start = rospy.Duration(duration)
    else:
        pt.time_from_start = joint_trajectory.points[-1].time_from_start + rospy.Duration(
            duration)
    pt.positions = positions
    joint_trajectory.points.append(pt)


def main():
    script_name = os.path.basename(__file__)
    node_name = os.path.splitext(script_name)[0]
    rospy.init_node(node_name)

    client = actionlib.SimpleActionClient(
        'joint_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo("Waiting for action server to start.")
    if client.wait_for_server(rospy.Duration(5)) == False:
        rospy.logerr('Failed to connect action server')
        exit(1)
    rospy.loginfo("Connected to action server")

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = make_joint_trajectory(1, ['joint_0', 'joint_1'])
    push_trajectory_point(goal.trajectory, 3, [
                          math.radians(90), math.radians(-90)])
    push_trajectory_point(goal.trajectory, 3, [
                          math.radians(-90), math.radians(90)])
    push_trajectory_point(goal.trajectory, 3, [
                          math.radians(0), math.radians(0)])

    rospy.loginfo("Sending goal")
    client.send_goal(goal)
    rospy.loginfo("Waiting for result...")
    finished = client.wait_for_result(rospy.Duration(10.0))

    if finished:
        state = client.get_state()
        rospy.loginfo("Action finished: %s" %
                      (actionlib.GoalStatus.to_string(state)))
        result = client.get_result()
        if result:
            rospy.loginfo("Result: Error code: %d, Error string: %s" %
                          (result.error_code, result.error_string))
    else:
        rospy.loginfo("Action did not finish before timeout.")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
