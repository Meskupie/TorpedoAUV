#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <Eigen/Geometry>

#define TF_FIXED_FRAME "/map"

int main(int argc, char **argv){
	//Initialize the ROS framework
    ros::init(argc,argv,"testing_node");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("marker_test", 5);

    ros::Time begin = ros::Time::now();
    double speed = 1;

    while(true){


        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.header.frame_id = TF_FIXED_FRAME;
        marker.pose.position.x = 1;
        marker.pose.position.y = 1;
        marker.pose.position.z = 1;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;
        marker.scale.x = 0.5;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.a = 1.0;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker_pub.publish(marker);
    }

    return 0;
}
