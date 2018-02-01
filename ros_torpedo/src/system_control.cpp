#include <ros/ros.h>

#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <Eigen/Geometry>

#include <vector>

#include <iostream>
#include <fstream>
#include <string>

#include <ros_torpedo/map_target.h>
#include <ros_torpedo/map_targets.h>

#include <resource_retriever/retriever.h>

#define MAPS_FILE_LOCATION "/home/meskupie/catkin_ws/src/TorpedoAUV/ros_torpedo/maps/"
#define MAP_MARKER_SIZE 0.05


ros_torpedo::map_targets load_map_file(std::string file_name){
    ros_torpedo::map_targets map_vector;
    std::fstream map_file((MAPS_FILE_LOCATION+file_name+".txt").c_str());
    std::string line;
    if(map_file.is_open()){
        ros_torpedo::map_target temp_target;
        float temp_size;
        map_file >> temp_size;
        map_vector.size = (int)temp_size;
        map_file >> map_vector.scale;
        ROS_INFO("%d,%f",map_vector.size,map_vector.scale);
        for(int i = 0; i < map_vector.size; i++){
            //int dump;
            //map_file >> dump;
            map_file >> temp_target.x;
            map_file >> temp_target.y;
            map_file >> temp_target.z;
            map_file >> temp_target.id;
            temp_target.x *= map_vector.scale;
            temp_target.y *= map_vector.scale;
            temp_target.z *= map_vector.scale;
            temp_target.id -= 48;
            map_vector.map_targets.push_back(temp_target);
        }
        map_file.close();
    }
    else{
        ROS_ERROR("Map failed to load");
    }
    return(map_vector);
}

void publish_map_markers(ros::NodeHandle n, ros_torpedo::map_targets map_vector){   
    ros::Publisher map_marker_1_pub = n.advertise<visualization_msgs::Marker>("map_marker_1", 5);
    ros::Publisher map_marker_2_pub = n.advertise<visualization_msgs::Marker>("map_marker_2", 5);
    ros::Publisher map_marker_3_pub = n.advertise<visualization_msgs::Marker>("map_marker_3", 5);
    ros::Publisher map_marker_4_pub = n.advertise<visualization_msgs::Marker>("map_marker_4", 5);
    visualization_msgs::Marker marker_1;
    visualization_msgs::Marker marker_2;
    visualization_msgs::Marker marker_3;
    visualization_msgs::Marker marker_4;

    marker_1.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_1.header.frame_id = "/inertial";
    marker_1.scale.x = MAP_MARKER_SIZE;
    marker_1.scale.y = MAP_MARKER_SIZE;
    marker_1.scale.z = MAP_MARKER_SIZE;
    marker_1.color.a = 1.0;
    marker_1.color.r = 1.0;
    marker_1.color.g = 1.0;
    marker_1.color.b = 0.0;
    
    marker_2.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_2.header.frame_id = "/inertial";
    marker_2.scale.x = MAP_MARKER_SIZE;
    marker_2.scale.y = MAP_MARKER_SIZE;
    marker_2.scale.z = MAP_MARKER_SIZE;
    marker_2.color.a = 1.0;
    marker_2.color.r = 1.0;
    marker_2.color.g = 0.2;
    marker_2.color.b = 0.8;

    marker_3.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_3.header.frame_id = "/inertial";
    marker_3.scale.x = MAP_MARKER_SIZE;
    marker_3.scale.y = MAP_MARKER_SIZE;
    marker_3.scale.z = MAP_MARKER_SIZE;
    marker_3.color.a = 1.0;
    marker_3.color.r = 0.0;
    marker_3.color.g = 1.0;
    marker_3.color.b = 0.0;

    marker_4.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_4.header.frame_id = "/inertial";
    marker_4.scale.x = MAP_MARKER_SIZE;
    marker_4.scale.y = MAP_MARKER_SIZE;
    marker_4.scale.z = MAP_MARKER_SIZE;
    marker_4.color.a = 1.0;
    marker_4.color.r = 1.0;
    marker_4.color.g = 1.0;
    marker_4.color.b = 1.0;


    geometry_msgs::Point p;
    for(std::vector<ros_torpedo::map_target>::iterator i = map_vector.map_targets.begin(); i != map_vector.map_targets.end(); i++){
        p.x = i->x;
        p.y = i->y;
        p.z = i->z;
        switch(i->id){
        case 1:
            //ROS_INFO("1, x:%f, y:%f, z:%f, id:%d",i->x,i->y,i->z,i->id);
            marker_1.points.push_back(p);
            break;
        case 2:
            //ROS_INFO("2, x:%f, y:%f, z:%f, id:%d",i->x,i->y,i->z,i->id);
            marker_2.points.push_back(p);
            break;
        case 3:
            //ROS_INFO("3, x:%f, y:%f, z:%f, id:%d",i->x,i->y,i->z,i->id);
            marker_3.points.push_back(p);
            break;
        case 4:
            //ROS_INFO("4, x:%f, y:%f, z:%f, id:%d",i->x,i->y,i->z,i->id);
            marker_4.points.push_back(p);
            break;
        default:
            ROS_ERROR("The map contained incorect IDs");
            break;
        }
    }

    map_marker_1_pub.publish(marker_1);
    map_marker_2_pub.publish(marker_2);
    map_marker_3_pub.publish(marker_3);
    map_marker_4_pub.publish(marker_4);
}

/*
void publish_map_stl(ros::NodeHandle n){
    ros::Publisher map_stl_pub = n.advertise<visualization_msgs::Marker>("map_stl", 5);
    visualization_msgs::Marker stl;

    stl.header.frame_id = "/inertial";
    stl.header.stamp = ros::Time();
    stl.type = visualization_msgs::Marker::MESH_RESOURCE;
    stl.pose.position.x = 0;
    stl.pose.position.y = 0;
    stl.pose.position.z = 0;
    stl.pose.orientation.x = 0.0;
    stl.pose.orientation.y = 0.0;
    stl.pose.orientation.z = 0.0;
    stl.pose.orientation.w = 1.0;
    stl.scale.x = 1;
    stl.scale.y = 1;
    stl.scale.z = 1;
    stl.color.a = 1.0; // Don't forget to set the alpha!
    stl.color.r = 1.0;
    stl.color.g = 1.0;
    stl.color.b = 1.0;
    stl.mesh_resource = "package://ros_torpedo/maps/course.stl";

    map_stl_pub.publish(stl);
}*/

int main(int argc, char **argv){
	//Initialize the ROS framework
    ros::init(argc,argv,"testing_node");
    ros::NodeHandle n;
    ros::Publisher map_vector_pub = n.advertise<ros_torpedo::map_targets>("map_vector", 5);

    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0,0,0));
    transform.setRotation(tf::Quaternion(1,0,0,0));

    ros_torpedo::map_targets map = load_map_file("rough_course_map");

    while(true){
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/fixed_frame", "/inertial"));
        publish_map_markers(n,map);
        map_vector_pub.publish(map);
    }

    
    return 0;
}
