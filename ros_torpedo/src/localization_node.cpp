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

class ParticleClass{
public:
    void iterateState(Eigen::Matrix<double, 12, 12>, Eigen::Matrix<double, 12, 6>, Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 12, 1>);
    void randomizeState(Eigen::Matrix<double, 12, 1>);

    tf::transform inertial_pose;
    Eigen::Matrix<double, 12, 1> body_pose_cur;
    Eigen::Matrix<double, 12, 1> body_pose_prev;
    Eigen::Matrix<double, 6, 1> delta_pose;
    
    double weight;
private:

}
void ParticleClass::iterateState(Eigen::Matrix<double, 12, 12> A, Eigen::Matrix<double, 12, 6> B, Eigen::Matrix<double, 6, 1> u, Eigen::Matrix<double, 12, 1> std){
    // Propogate the motion model forward one time step
    body_pose_cur = A*body_pose_prev + B*u;
    this.randomizeState(std);

    // Update the particle inertial state through a delta transform from the body frame iteration
    tf::Transform delta;
    delta.setOrigin(tf::Vector3(body_pose_cur(0),body_pose_cur(2),body_pose_cur(4)));
    delta.setRotation(tf::Quaternion(body_pose_cur(10),body_pose_cur(8),body_pose_cur(6)));
    inertial_pose *= delta;

    // Zero out deltas 
    body_pose_cur(0) = 0;
    body_pose_cur(2) = 0;
    body_pose_cur(4) = 0;
    body_pose_cur(6) = 0;
    body_pose_cur(8) = 0;
    body_pose_cur(10) = 0;
    body_pose_prev = body_pose_cur;
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
    body_pose_cur += output
}


class LocalizationClass{
public:
    LocalizationClass(ros::NodeHandle _n);

private:
    void systemParametersCallback(const ros_torpedo::system_parameters);
    //void imuCallback(const geometry_msgs::twist);
    //void cameraPointsCallback(ros_torpedo::camera_points);
    //void systemStateCallback(const ros_torpedo::system_state);

    ros::NodeHandle n;
    ros::Subscriber system_parameters_sub;

    tf::TransformListener listener;

    Eigen::Matrix<double, 6, 1> u_prev;

    Eigen::Matrix<double, 12, 12> model_A;
    Eigen::Matrix<double, 12, 6> model_B;

    Eigen::Matrix<double, 12, 1> motion_noise_model;
    Eigen::Matrix<double, 12, 1> initial_noise_model;
};


LocalizationClass::LocalizationClass(ros::NodeHandle _n){
        // grab node handle
        n = _n;

        //Subscribe to the desired topics and assign callbacks
        system_parameters_sub = n.subscribe("/system_parameters", 1, &LocalizationClass::systemParametersCallback, this);

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
// - start with a particle cloud at x0
//     - Assume the vehicle has no velocity
//     - Propagate particles forward with large noise model 
// - while true

//         if camera_data available
//             === repeat from here if running multiple scan matches 
//             - find vector and orientation to both cameras
//             - calculate quaternion of each target in the inertial frame
//             - rotate a unit direction vector by the target quaternion
//             For each perception target aquired:
//                 For each point in the map
//                     - calculate projection of the point to the line
//                     - Disregard if projection scaling is negative
//                 - pick and store the point on the map with the minimum orthoganal distance
//             if scanmatch
//                 - build a point cloud with closest points along the target line
//                 - run pcl::icp and grab the transform
//                 - rotate the transform position vector by the inertial orientation
//                 - add the rotated delta vector to the inertial position
//                 - multiply quaternions to apply a delta orientation update
//                 - repeat from "===" if desired
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
