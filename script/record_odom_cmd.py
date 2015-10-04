#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand

def odom_callback(data):
    p = data.pose.pose.position
    v = data.twist.twist.linear
    t = data.header.stamp.to_sec()

    print "0 {0} {1} {2} {3} {4} {5} {6}".format( \
            p.x, p.y, p.z, v.x, v.y, v.z, t)

def cmd_callback(data):
    p = data.position
    v = data.velocity
    t = data.header.stamp.to_sec()
    print "1 {0} {1} {2} {3} {4} {5} {6}".format( \
            p.x, p.y, p.z, v.x, v.y, v.z, t)

def listener():
    rospy.init_node("listener");
    rospy.Subscriber("/odom", Odometry, odom_callback);
    rospy.Subscriber("/position_cmd", PositionCommand, cmd_callback);
    rospy.spin()

if __name__ == "__main__":
    listener()
