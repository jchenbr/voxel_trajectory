#!/usr/bin/env python

import rospy
from visualization_msgs.msg import MarkerArray, Marker

rospy.init_node("listener", anonymous = True)
mk_pub = rospy.Publisher("floor_inflated_path", MarkerArray, queue_size = 2)
p_pub = rospy.Publisher("argument_path", MarkerArray, queue_size = 2)

def marker_callback(data):
    for msg in data.markers:
        msg.scale.z = 0.02;
        msg.pose.position.z = 0.0;
        msg.color.r = 1.0;
        msg.color.g = 1.0;
        msg.color.b = 1.0;
        msg.color.a = 0.7;
    mk_pub.publish(data);

def path_callback(data):
    for msg in data.markers:
        msg.color.a = 0.2;
    p_pub.publish(data);

if __name__ == "__main__":
    rospy.Subscriber("/trajectory_generator/inflated_path", MarkerArray, marker_callback)
    rospy.Subscriber("/trajectory_generator/voxel_path", MarkerArray, path_callback)
    rospy.spin();

