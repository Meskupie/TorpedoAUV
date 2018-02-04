#include <ros/ros.h>

#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Geometry>

#include <vector>

#include <iostream>
#include <fstream>
#include <string>

#include <ros_torpedo/map_target.h>
#include <ros_torpedo/map_targets.h>
#include <ros_torpedo/system_parameters.h>


#define MAPS_FILE_LOCATION "/home/meskupie/catkin_ws/src/TorpedoAUV/ros_torpedo/maps/"
#define CONTROLLER_FILE_LOCATION "/home/meskupie/catkin_ws/src/TorpedoAUV/ros_torpedo/controllers/"
#define PATHS_FILE_LOCATION "/home/meskupie/catkin_ws/src/TorpedoAUV/ros_torpedo/paths/"
#define MAP_MARKER_SIZE 0.05

class Parameters{
public:
    Parameters(ros::NodeHandle _n);
    void generate();
    void publish();

    std::string controller_filename;
    std::string map_filename;
    std::string path_filename;

private:
    void loadMapFile();
    void loadControllerFile();

    ros::NodeHandle n;
    ros::Publisher system_parameters_pub;
    ros::Publisher map_marker_1_pub;
    ros::Publisher map_marker_2_pub;
    ros::Publisher map_marker_3_pub;
    ros::Publisher map_marker_4_pub;
    tf::TransformBroadcaster fixed_frame_br;
    tf::Transform fixed_frame_tr;

    visualization_msgs::Marker marker_1;
    visualization_msgs::Marker marker_2;
    visualization_msgs::Marker marker_3;
    visualization_msgs::Marker marker_4;

    ros_torpedo::system_parameters system_parameters;

    std::vector<double> model_A_msg;
    std::vector<double> model_B_msg;
    std::vector<double> lqr_K_msg;

    Eigen::Matrix<double, 12, 12> model_A;
    Eigen::Matrix<double, 12, 6> model_B;
    Eigen::Matrix<double, 6, 12> lqr_K;
};

Parameters::Parameters(ros::NodeHandle _n){
    // Grab node handle
    n = _n;

    // Setup topics to Publish from this node
    system_parameters_pub   = n.advertise<ros_torpedo::system_parameters>("system_parameters", 1, true);
    map_marker_1_pub = n.advertise<visualization_msgs::Marker>("map_marker_1", 1, true);
    map_marker_2_pub = n.advertise<visualization_msgs::Marker>("map_marker_2", 1, true);
    map_marker_3_pub = n.advertise<visualization_msgs::Marker>("map_marker_3", 1, true);
    map_marker_4_pub = n.advertise<visualization_msgs::Marker>("map_marker_4", 1, true);

    // Setup tf
    fixed_frame_tr.setOrigin(tf::Vector3(0,0,0));
    fixed_frame_tr.setRotation(tf::Quaternion(1,0,0,0));
}

void Parameters::generate(){
    this->loadMapFile();
}

void Parameters::publish(){
    system_parameters_pub.publish(system_parameters);
    fixed_frame_br.sendTransform(tf::StampedTransform(fixed_frame_tr, ros::Time::now(), "/fixed_frame", "/inertial"));

    map_marker_1_pub.publish(marker_1);
    map_marker_2_pub.publish(marker_2);
    map_marker_3_pub.publish(marker_3);
    map_marker_4_pub.publish(marker_4);
}

void Parameters::loadMapFile(){
    std::fstream map_file((MAPS_FILE_LOCATION+map_filename+".txt").c_str());
    if(map_file.is_open()){
        ros_torpedo::map_target temp_target;
        double temp_size;
        map_file >> temp_size;
        system_parameters.map_vector.size = (int)temp_size;
        map_file >> system_parameters.map_vector.scale;
        for(int i = 0; i < system_parameters.map_vector.size; i++){
            map_file >> temp_target.x;
            map_file >> temp_target.y;
            map_file >> temp_target.z;
            map_file >> temp_target.id;
            temp_target.x *= system_parameters.map_vector.scale;
            temp_target.y *= system_parameters.map_vector.scale;
            temp_target.z *= system_parameters.map_vector.scale;
            temp_target.id -= 48;
            system_parameters.map_vector.map_targets.push_back(temp_target);
        }
        map_file.close();
    }
    else{
        ROS_ERROR("Map file failed to load");
    }

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
    for(std::vector<ros_torpedo::map_target>::iterator i = system_parameters.map_vector.map_targets.begin(); i != system_parameters.map_vector.map_targets.end(); i++){
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
}

void Parameters::loadControllerFile(){
    std::fstream controller_file((CONTROLLER_FILE_LOCATION+controller_filename+".txt").c_str());
    if(controller_file.is_open()){
        double temp;
        for(int i = 0; i < 144; i++){
            controller_file >> temp;
            system_parameters.model_A.push_back(temp);
        }
        for(int i = 0; i < 72; i++){
            controller_file >> temp;
            system_parameters.model_B.push_back(temp);
        }
        for(int i = 0; i < 72; i++){
            controller_file >> temp;
            system_parameters.lqr_K.push_back(temp);
        }
        controller_file.close();
    }
    else{
        ROS_ERROR("Controller file failed to load");
    }
}

int main(int argc, char **argv){
	//Initialize the ROS framework
    ros::init(argc,argv,"system_control_node");
    ros::NodeHandle n;

    Parameters parameters(n);
    parameters.map_filename = "rough_course_map";
    parameters.controller_filename = "rough_controller_data";
    parameters.generate();

    tf::Transform start;
    start.setRotation(tf::Quaternion(0.271,-0.653,0.271,0.653));
    start.setOrigin(tf::Vector3(1,1,0));

    tf::Transform delta;
    delta.setRotation(tf::Quaternion(0,0.383,0,0.924));
    delta.setOrigin(tf::Vector3(1,0,0));
    

    tf::Transform finish = start*delta;

    ROS_INFO("%f,%f,%f",finish.getOrigin().x(),finish.getOrigin().y(),finish.getOrigin().z());
    ROS_INFO("%f,%f,%f,%f",finish.getRotation().w(),finish.getRotation().x(),finish.getRotation().y(),finish.getRotation().z());

    Eigen::Matrix<double,3,3> test;
    test << 0,0,0,0,0,0,0,0,0;
    for(int i = 0; i < 9; i++){
        test(i) = i;
    }

    std::cout << test << std::endl;

    //while(true){
    //    parameters.publish();
    //}
    return 0;
}
