#!/usr/bin/env python
import math
import os

import rospy
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
    pub = rospy.Publisher('joint_controller/joint_trajectory',
                          JointTrajectory, queue_size=1)
    rospy.sleep(3)

    msg = make_joint_trajectory(1, ['right_arm_1', 'right_arm_2'])
    push_trajectory_point(msg, 0.5, [math.radians(90),
                                   math.radians(-90)])
    push_trajectory_point(msg, 0.5, [math.radians(-90),
                                   math.radians(90)])
    push_trajectory_point(msg, 0.5, [math.radians(0),
                                   math.radians(0)])

    rospy.loginfo("Sending trajectory")
    pub.publish(msg)
    rospy.sleep(9)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
