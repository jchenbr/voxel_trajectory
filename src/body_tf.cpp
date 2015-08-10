#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

void tfOdomCallback(const nav_msgs::Odometry odom)
{
    static tf::TransformBroadcaster br;
    tf::Transform tf;
    tf.setOrigin(tf::Vector3(
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                odom.pose.pose.position.z));
    tf.setRotation(tf::Quaternion(
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w));
    br.sendTransform(tf::StampedTransform(tf, odom.header.stamp, "map", "body"));
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "quad_tf");

    ros::NodeHandle handle("~");
    auto sub = handle.subscribe("odometry", 20, &tfOdomCallback);

    ros::spin();
    return 0;
}
