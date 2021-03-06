/* ##PSUDO CODE##

notes:
- Each particle will have an inertial position (x,y,z,qx,qy,qz,qw). This will be a tf::transform
- Each particle will have a body state (x',y',z',r',p',w') with relative positions (x,y,z,r,p,w)
    - relative positions are always 0 at t prev

start:
- start with a particle cloud at x0
    - Assume the vehicle has no velocity
    - Propagate particles forward with large noise model 
- while true
    - For each particle:
        - Using the most recent command measured from motors
        - propogate the motion model forward one time step
        - add gausian noise based on motion noise profile
        * results will be a new state vector in the body frame
        - rotate the delta position vector by the inertial orientation
        - add the rotated delta vector to the inertial position
        - multiply quaternions to apply a delta orientation update
        - zero out deltas 
        * partical now has a new global position

        if camera_data available
            === repeat from here if running multiple scan matches 
            - find vector and orientation to both cameras
            - calculate quaternion of each target in the inertial frame
            - rotate a unit direction vector by the target quaternion
            For each perception target aquired:
                For each point in the map
                    - calculate projection of the point to the line
                    - Disregard if projection scaling is negative
                - pick and store the point on the map with the minimum orthoganal distance

            if scanmatch
                - build a point cloud with closest points along the target line
                - run pcl::icp and grab the transform
                - rotate the transform position vector by the inertial orientation
                - add the rotated delta vector to the inertial position
                - multiply quaternions to apply a delta orientation update
                - repeat from "===" if desired
        
*/

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_ros/point_cloud.h>

#include "common/statistics.cpp"

#include <Eigen/Geometry>

#include <vector>

#include <ros_torpedo/system_parameters.h>

#define CAMERA_ARM_LENGTH 0.1

struct CameraTarget{
    tf::Transform target_transform;
    tf::Vector3 target_unit_vector;
    tf::Vector3 estimated_position;
    double scale;
    double distance;
    double score;
    uint corresponding_map_index;
};


class ParticleClass{
public:
    void iterateState(Eigen::Matrix<double, 12, 12>, Eigen::Matrix<double, 12, 6>, Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 12, 1>);
    void randomizeState(Eigen::Matrix<double, 12, 1>);

    tf::transform inertial_pose;
    Eigen::Matrix<double, 12, 1> x_body_cur;
    Eigen::Matrix<double, 12, 1> x_body_prev;
    
    double weight;
private:

}

ParticleClass::ParticleClass(){

}

void ParticleClass::iterateState(Eigen::Matrix<double, 12, 12> A, Eigen::Matrix<double, 12, 6> B, Eigen::Matrix<double, 6, 1> u, Eigen::Matrix<double, 12, 1> std){
    // Propogate the motion model forward one time step
    x_body_cur = A*x_body_prev + B*u;
    this.randomizeState(std);

    // Update the particle inertial state through a delta transform from the body frame iteration
    tf::Transform delta;
    delta.setOrigin(tf::Vector3(x_body_cur(0),x_body_cur(2),x_body_cur(4)));
    delta.setRotation(tf::Quaternion(x_body_cur(10),x_body_cur(8),x_body_cur(6)));
    inertial_pose *= delta;

    // Zero out deltas 
    x_body_cur(0) = 0;
    x_body_cur(2) = 0;
    x_body_cur(4) = 0;
    x_body_cur(6) = 0;
    x_body_cur(8) = 0;
    x_body_cur(10) = 0;
    x_body_prev = x_body_cur;
}


void ParticleClass::randomizeState(Eigen::Matrix<double, 12, 1> std){
    Eigen::Matrix<double, 12 ,1> noise;
    double n = 0;
    int iterations = 12;
    for(int i = 0; i<12; i++){
        // emulate a normal distribution
        for(int i = 0; i < iterations; i++){
            n += (rand()%10000)/10000.0f;
        }
        noise(i) = (n-double(iterations)/2.0f)*std(i);
    }
    x_body_cur += output
}

//================================================

class LocalizationClass{
public:
    LocalizationClass(ros::NodeHandle _n);

private:
    void systemParametersCallback(const ros_torpedo::system_parameters);
    void systemStateCallback(const ros_torpedo::system_state);

    ros::NodeHandle n;
    ros::Subscriber system_parameters_sub;
    ros::Subscriber system_state_sub;
    ros::Subscriber camera_targets_sub;

    tf::TransformListener listener;

    Eigen::Matrix<double, 12, 1> x_initial;
    Eigen::Matrix<double, 6, 1> u_prev;
    Eigen::Matrix<double, 12, 12> model_A;
    Eigen::Matrix<double, 12, 6 > model_B;
    Eigen::Matrix<double, 6 , 12> lqr_K;

    Eigen::Matrix<double, 12, 1> initial_noise_model;

    Eigen::Matrix<double, 12, 1> motion_noise_model;
    Eigen::Matrix<double, 12, 1> initial_noise_model;

    ros_torpedo::map_targets map_vector;

    bool pose_lock
    bool allow_parameter_chage;

    std::vector<CameraTarget> target_vector;
    tf::Transform front_camera_transform;
    tf::TRansform rear_camera_transform;

    bool new_camera_available;
    bool data_valid;

};

LocalizationClass::LocalizationClass(ros::NodeHandle _n){
    // grab node handle
    n = _n;

    //Subscribe to the desired topics and assign callbacks
    system_parameters_sub = n.subscribe("/system_parameters", 1, &LocalizationClass::systemParametersCallback, this);
    system_state_sub = n.subscribe("/system_state", 1, &LocalizationClass::systemStateCallback, this);
    camera_targets_sub = n.subscribe("/camera_targets", 1, &LocalizationClass::cameraTargetsCallback, this);


    //Setup topics to Publish from this node

    //Initialize variables
    initial_noise_model <<  0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0;

    motion_noise_model <<   0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0; 

    // Setup the camera transforms
    front_camera_transform.setOrigin(CAMERA_ARM_LENGTH,0,0);
    front_camera_transform.setRotation(0,0,0,1)
    rear_camera_transform.setOrigin(-CAMERA_ARM_LENGTH,0,0);
    rear_camera_transform.setRotation(0,0,1,0)

    new_camera_available = false;        
  
}

void systemParametersCallback(const ros_torpedo::system_parameters parameters){
    if(allow_parameter_chage == true){
        map_vector = parameters.map_vector;
        for(int i = 0; i < 12; i++){
            x_initial(i) = parameters.x_initial.at(i); }
        for(int i = 0; i < 144; i++){
            model_A(i) = parameters.model_A.at(i); }
        for(int i = 0; i < 72; i++){
            model_B(i) = parameters.model_B.at(i); }
        for(int i = 0; i < 72; i++){
            lqr_K(i) = parameters.lqr_K.at(i); }
    }
}

void LocalizationClass::systemStateCallback(const ros_torpedo::system_state){
    if(pose_lock == false){
        for(std::vector<ParticalClass>::iterator i = partical_cloud.begin(); i != partical_cloud.end(); i++){
            *i.randomizeState(initial_noise_model);
        }
    } else {
        for(std::vector<ParticalClass>::iterator i = partical_cloud.begin(); i != partical_cloud.end(); i++){
            *i.updateState(model_A,model_B,u_prev,motion_noise_model);
        }
    }
    // Only proceed if there is new camera data waiting
    if(camera_targets_ready){
        // Iterate through all particles
        for(std::vector<ParticalClass>::iterator i = partical_cloud.begin(); i != partical_cloud.end(); i++){
            tf::Transform front_camera_inertial = i->inertial_pose*front_camera_transform;
            tf::Transform rear_camera_inertial = i->inertial_pose*rear_camera_transform;
            // Iterate through all front and rear targets
            // The goal is to associate points on the map with targets from perception
            for(std::vector<tf::Quaternion>::iterator j = front_target_lines.begin(); j != front_target_lines.end(); j++){
                // find the closest inertial point for this target 
                tf::Vector3 closest_point = findClosestPoint(front_camera_intertial, *j);
                // Check if our match was a dud
                if(closest_point == tf::Vector3(0,0,0)){

                }
            }
        }
    }

//             if scanmatch
//                 - build a point cloud with closest points along the target line
//                 - run pcl::icp and grab the transform
//                 - rotate the transform position vector by the inertial orientation
//                 - add the rotated delta vector to the inertial position
//                 - multiply quaternions to apply a delta orientation update
//                 - repeat from "===" if desired
}

void LocalizationClass::cameraTargetsCallback(ros_torpedo::camera_targets _targets){
    target_vector.clear()
    CameraTarget target_container
    for(std::vector<ros_torpedo::map_target>::iterator i = _targets.front_targets.begin(); i != _targets.front_targets.end(); i++){
        target_container.setOrigin(0,0,0);
        target_container.setRotation(tf::quaternion(i->x,i->y,i->z,i->w));
        target_container *= front_camera_transform;
        target_vector.push_back(target_container);
    }
    for(std::vector<ros_torpedo::map_target>::iterator i = _targets.rear_targets.begin(); i != _targets.rear_targets.end(); i++){
        target_container.setOrigin(0,0,0);
        target_container.setRotation(tf::quaternion(i->x,i->y,i->z,i->w));
        target_container *= rear_camera_transform;
        target_vector.push_back(target_container);
    }
    if(!target_vector.empty()){
        data_valid = false;
        new_data_available = true;
    } else {
        ROS_ERROR("No camera targets visible");
    }
}

void LocalizationClass::findTargetCorrespondences(){
    for(std::vector<std::vector<CameraTarget> >::iterator i = target_vector.begin(); i != target_vector.end(); i++){
        i->score = 0;
        i->scale = 0;
        i->distance = 0;
        double min_score = 100000; //hopefully this is always big enough for initialization
        double min_index = -1;
        double count = 0;
        tf::Vector3 map_point_body;
        tf::Vector3 target_vector;
        std::vector<tf::Vector3> potential_closest_points;
        tf::Vector3 target_unit_vector = tf::Vector3(1,0,0)*(front_camera_inertial.getRotation()*(target_direction));
        // Look though all map targets to find the best one
        for(std::vector<ros_torpedo::map_targets>::iterator i = map_vector.begin(); i != map_vector.end(); i++){
            // Calculate a vector of this map target relative to the body frame of the camera
            map_point_body.setValue(i->map_target.x,i->map_target.y,i->map_target.z);
            map_point_body -= camera_inertial.getOrigin();
            // Calculate the unit target line vector scale. This is a projection to the map
            scale = map_point_body.dot(target_unit_vector)/target_unit_vector.dot(target_unit_vector);
            // If scale is negative, we are definately tracking the wrong target...
            target_vector.setOrigin(0,0,0);
            if(scale > 0){
                target_vector = scale*target_unit_vector;
                distance = map_point_body.distance2(target_vector);
                // Score is a combination of the error in the closest point and how close the target is
                // The idea is to prevent interference from far away targets which are less likely to be the match
                score = distance+scale/500;
                if(score < min_score){
                    min_score = score;
                    min_index = count;
                }
            }
            potential_closest_points.push_back(target_vector);
            count++;
        }
        // check to see if our guess is a dud
        if(min_index < 0 || min_score > 0.5){
            // technically this dud return is not out of bounds, but i'll ignore the unlikely case
            return tf::Vector3(0,0,0);
        }
        return potential_closest_points.at(min_index)+camera_inertial.getOrigin();
    }  
}

int main(int argc, char **argv){
	//Initialize the ROS framework
    ros::init(argc,argv,"localization_node");
    if (argc != 2){
        ROS_INFO("Please specify a map file name to load");
        return 1;
    }

    ros::NodeHandle n;
    LocalizationClass turtle(n,argv[1]);

    ros::spin();   //Check for new messages

    return 0;
}
