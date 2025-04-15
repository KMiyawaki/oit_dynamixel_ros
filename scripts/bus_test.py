#!/usr/bin/env python
import os

import rospy

from oit_dynamixel_ros.msg import ServoCommand, ServoState


class BusTest(object):
    def __init__(self, servo_command_topic='dynamixel_bus/servo_command', servo_state_topic='dynamixel_bus/servo_state'):
        self.servo_command_seq = 0
        self.servo_commands = []
        self.servo_commands.append(
            self.make_servo_command([1, 2], 3, [90, -90]))
        self.servo_commands.append(
            self.make_servo_command([1, 2], 3, [-90, 90]))
        self.servo_commands.append(self.make_servo_command([1, 2], 3, [0, 0]))

        self.servo_command_pub = rospy.Publisher(
            servo_command_topic, ServoCommand, queue_size=1)
        self.servo_state_sub = rospy.Subscriber(
            servo_state_topic, ServoState, self.servo_state_cb, queue_size=1)

    def make_servo_command(self, ids, duration, angles):
        com = ServoCommand()
        com.ids = ids
        com.duration = duration
        com.angles = angles
        com.sender = rospy.get_name()
        return com

    def servo_state_cb(self, msg):
        rospy.logdebug(msg)

    def spin(self):
        idx = self.servo_command_seq % len(self.servo_commands)
        c = self.servo_commands[idx]
        c.header.seq = self.servo_command_seq
        c.header.stamp = rospy.Time.now()
        self.servo_command_pub.publish(c)
        rospy.loginfo('Sent servo command')
        self.servo_command_seq = self.servo_command_seq + 1
        rospy.sleep(c.duration)


def main():
    script_name = os.path.basename(__file__)
    node_name = os.path.splitext(script_name)[0]
    rospy.init_node(node_name)
    node = BusTest()
    rospy.sleep(3)  # すぐにコマンドを送ると無視される
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        node.spin()
        r.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
