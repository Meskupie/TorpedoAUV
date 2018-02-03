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
        



    tf::Matrix3x3 test;
    test.setRotation(tf::Quaternion(0.5,-0.5,0.5,-0.5));
    tf::Vector3 test2;
    test2.setValue(0,0,1);

    tf::Vector3 test3;
    test3 = test2*test;

    ROS_INFO("%f,%f,%f",test3.x(),test3.y(),test3.z());
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
    tf::transform inertial_pose;
    Eigen::Matrix<double, 12, 1> body_pose_cur;
    Eigen::Matrix<double, 12, 1> body_pose_prev;
    Eigen::Matrix<double, 6, 1> delta_pose;
    
    double weight;
private:

}


class LocalizationClass{
public:
    LocalizationClass(ros::NodeHandle _n);

private:
    ros::NodeHandle n;
    
    ros::Subscriber system_parameters_sub;

    // tf listener to keep track of frames
    tf::TransformListener listener;


    Eigen::Matrix<double, 6, 1> u_prev;

    Eigen::Matrix<double, 12, 12> model_A;
    Eigen::Matrix<double, 12, 6> model_B;

    Eigen::Matrix<double, 12, 1> motion_noise_model;
    Eigen::Matrix<double, 12, 1> initial_noise_model;

    void systemParametersCallback(const ros_torpedo::system_parameters);
    //void imuCallback(const geometry_msgs::twist);
    //void cameraPointsCallback(ros_torpedo::camera_points);
    //void systemStateCallback(const ros_torpedo::system_state);
};


LocalizationClass::LocalizationClass(ros::NodeHandle _n){
        // grab node handle
        n = _n;

        //Subscribe to the desired topics and assign callbacks
        system_parameters_sub = n.subscribe("/system_parameters", 1, &LocalizationClass::systemParametersCallback, this);

        //Setup topics to Publish from this node]

        // Set initial states
        prev_mean  << 0,0,0;
        prev_cov   << 1,0,0,
                      0,1,0,
                      0,0,1;
        cur_input  << 0,0;

        // Set matricies
        ekf_C = Eigen::Matrix<double, 3, 3>::Identity();

        // Set model and sensor coveriances
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
    if(pose_lock == true){

        for(std::vector<partical>::iterator i = partical_cloud.begin(); i != partical_cloud.end(); i++){
            // Propogate the motion model forward one time step
            Eigen::Matrix(double, 12, 1) motion_noise;
            motion_noise = 

            i->body_pose_cur = model_A*i->body_pose_prev + model_B*u_prev + motion_noise;
            delta_pose <<   body_pose_cur(1),
                            body_pose_cur(3),
                            body_pose_cur(5),
                            body_pose_cur(7),
                            body_pose_cur(9),
                            body_pose_cur(11);

        }


    }
// - start with a particle cloud at x0
//     - Assume the vehicle has no velocity
//     - Propagate particles forward with large noise model 
// - while true
//     - For each particle:
//         - Using the most recent command measured from motors
//         - propogate the motion model forward one time step
//         - add gausian noise based on motion noise profile
//         * results will be a new state vector in the body frame
//         - rotate the delta position vector by the inertial orientation
//         - add the rotated delta vector to the inertial position
//         - multiply quaternions to apply a delta orientation update
//         - zero out deltas 
//         * partical now has a new global position
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
