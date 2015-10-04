#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import Point

rospy.init_node("filter_traj");
wp_traj_pub = rospy.Publisher("wp_traj_vis", Marker, queue_size = 2);
nf_traj_pub = rospy.Publisher("nf_traj_vis", Marker, queue_size = 2);
if_traj_pub = rospy.Publisher("if_traj_vis", Marker, queue_size = 2);

def waypoints_traj_callback(data):
    data.color.b = 255.0/255;
    data.color.r = 0;
    data.color.g = 85.0/255;
    data.color.a = 0.7;
    wp_traj_pub.publish(data);
    tot_len = 0.0;
    for i in range(1, len(data.points)):
        a = data.points[i - 1];
        b = data.points[i - 2];
        tot_len += ((a.x - b.x)**2 + (a.y - b.y)**2)**0.5;
    print "Waypoints trajectory length :", tot_len;
 
def no_inflation_traj_callback(data):
    data.color.b = 50.0/255;
    data.color.r = 50.0/255;
    data.color.g = 224.0/255;
    data.color.a = 0.7;
    nf_traj_pub.publish(data);
    tot_len = 0.0;
    for i in range(1, len(data.points)):
        a = data.points[i - 1];
        b = data.points[i - 2];
        tot_len += ((a.x - b.x)**2 + (a.y - b.y)**2)**0.5;
    print "No inflation trajectory length :", tot_len;
 
def path_callback(data):
    msg = Marker();
    msg.ns = "SSSSSSS";
    msg.id = 1;
    msg.header = data.header;
    msg.action = msg.ADD;
    msg.type = msg.LINE_STRIP;
    msg.color.r = 0.0/255;
    msg.color.g = 0.0/255;
    msg.color.b = 255.0/255;
    msg.color.a = 0.7;
    msg.scale.x = 0.1;
    msg.scale.y = 0.1;
    msg.scale.z = 0.1;
    msg.pose.orientation.w = 1.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.position.x = 0.0;
    msg.pose.position.y = 0.0;
    msg.pose.position.z = 0.0;

    #print len(data.poses)
    
    for idx in range(1, len(data.poses)):
        now = data.poses[idx].pose.position;
        last = data.poses[idx - 1].pose.position;

        pt = Point()
        for i in range(10):
            pt.x = (now.x * i + last.x * (10 - i)) /10.0;
            pt.y = (now.y * i + last.y * (10 - i)) /10.0;
            pt.z = (now.z * i + last.z * (10 - i)) /10.0;
            msg.points.append(pt);

    if_traj_pub.publish(msg);

def traj_callback(data):
    tot_len = 0.0;
    for i in range(1, len(data.points)):
        a = data.points[i - 1];
        b = data.points[i - 2];
        tot_len += ((a.x - b.x)**2 + (a.y - b.y)**2)**0.5;
    print "Trajectory length :", tot_len;
    
def listener():
    rospy.Subscriber("/waypoints_trajectory_generator/traj_vis", Marker, waypoints_traj_callback);
    rospy.Subscriber("/no_inflation_trajectory_generator/trajectory_vis", Marker, no_inflation_traj_callback);
    rospy.Subscriber("/odom_visualization_ukf/path", Path, path_callback);
    rospy.Subscriber("/trajectory_generator/line_strip", Marker, traj_callback);
    rospy.spin();

if __name__ == "__main__":
    listener();
