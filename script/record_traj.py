#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

def traj_callback(data):
    output = [data.header.stamp.to_sec()]
    for pt in data.points:
        output.append(pt.x)

    for pt in data.points:
        output.append(pt.y)

    for pt in data.points:
        output.append(pt.z)

    print output
    
def listener():
    rospy.init_node("listener_traj");
    rospy.Subscriber("/trajectory_generator/line_strip", Marker, traj_callback);
    rospy.spin()

if __name__ == "__main__":
    listener()
