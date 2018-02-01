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

#define MAPS_FILE_LOCATION "/home/meskupie/catkin_ws/src/TorpedoAUV/ros_torpedo/maps/"
#define TF_FIXED_FRAME "/map"

ros_torpedo::map_targets load_map_file(std::string file_name){
    ros_torpedo::map_targets map_vector;
    std::fstream map_file((MAPS_FILE_LOCATION+file_name+".txt").c_str());
    std::string line;
    if(map_file.is_open()){
        ros_torpedo::map_target temp_target;
        int file_length;
        map_file >> file_length;
        for(int i = 0; i < file_length; i++){
            //int dump;
            //map_file >> dump;
            map_file >> temp_target.x;
            map_file >> temp_target.y;
            map_file >> temp_target.z;
            map_file >> temp_target.id;
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

int main(int argc, char **argv){
	//Initialize the ROS framework
    ros::init(argc,argv,"testing_node");
    ros::NodeHandle n;
    ros::Publisher map_vector_pub = n.advertise<ros_torpedo::map_targets>("map_vector", 5);

    ros_torpedo::map_targets map = load_map_file("test_map");
    for(int i = 0; i<5;i++){
        ROS_INFO("x:%f, y:%f, z:%f, id:%d",map.map_targets[i].x,map.map_targets[i].y,map.map_targets[i].z,map.map_targets[i].id);
    }
    map_vector_pub.publish(map);
    return 0;
}
