#ifndef _MARKERS
#define _MARKERS

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <vector>

#define TF_FIXED_FRAME "/absolute"

#define THIN_LINE_WIDTH 0.05
#define THICK_LINE_WIDTH 0.15

class Marker{
protected:
    ros::Publisher pose_pub;
    ros::Publisher ellipse_pub;
    ros::Publisher arc_pub;

public:
    Marker(ros::NodeHandle n, std::string marker_name){
        pose_pub    = n.advertise<visualization_msgs::Marker>(marker_name+"_pose", 5);
        ellipse_pub = n.advertise<visualization_msgs::Marker>(marker_name+"_covariance_ellipse", 5);
        arc_pub     = n.advertise<visualization_msgs::Marker>(marker_name+"_covariance_arc", 5);
    }
};


class PoseMarker: public Marker{
private:
    double r, g, b;
public:
    PoseMarker(ros::NodeHandle n, std::string marker_name, double _r, double _g, double _b)
    :
    Marker(n, marker_name)
	{
	    r = _r;
	    g = _g;
	    b = _b;
	}

    void updatePoseTransform(tf::Transform marker_transform){

    	tf::Vector3 translation = marker_transform.getOrigin();
        tf::Quaternion rotation = marker_transform.getRotation();

        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.header.frame_id = TF_FIXED_FRAME;
        marker.pose.position.x = translation.x();
        marker.pose.position.y = translation.y();
        marker.pose.position.z = translation.z();
        marker.pose.orientation.x = rotation.x();
        marker.pose.orientation.y = rotation.y();
        marker.pose.orientation.z = rotation.z();
        marker.pose.orientation.w = rotation.w();
        marker.scale.x = 0.5;
        marker.scale.y = THIN_LINE_WIDTH;
        marker.scale.z = THIN_LINE_WIDTH;
        marker.color.a = 1.0;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        pose_pub.publish(marker);
    }

    void updatePoseStateVector(Eigen::Matrix<double, 3, 1> marker_state){

    	tf::Quaternion rotation;
    	rotation.setEuler(0,0,marker_state(2));

    	visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.header.frame_id = TF_FIXED_FRAME;
        marker.pose.position.x = marker_state(0);
        marker.pose.position.y = marker_state(1);
        marker.pose.position.z = 0;
        marker.pose.orientation.x = rotation.x();
        marker.pose.orientation.y = rotation.y();
        marker.pose.orientation.z = rotation.z();
        marker.pose.orientation.w = rotation.w();
        marker.scale.x = 0.5;
        marker.scale.y = THIN_LINE_WIDTH;
        marker.scale.z = THIN_LINE_WIDTH;
        marker.color.a = 1.0;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        pose_pub.publish(marker);
    }

    void updateEllipseStateVector(Eigen::Matrix<double, 3, 1> marker_state, Eigen::Matrix<double,3,3> full_covariance, double number_of_stddev){

    	visualization_msgs::Marker marker;
    	marker.type = visualization_msgs::Marker::LINE_STRIP;
    	marker.header.frame_id = TF_FIXED_FRAME;
        marker.scale.x = THIN_LINE_WIDTH;
        marker.scale.y = THIN_LINE_WIDTH;
        marker.scale.z = THIN_LINE_WIDTH;
        marker.color.a = 1.0;
        marker.color.r = r;        
        marker.color.g = g;
        marker.color.b = b; 

        // find eigevectors and values
        Eigen::Matrix<double,2,2> covariance;
        covariance << full_covariance(0,0),full_covariance(0,1),full_covariance(1,0),full_covariance(1,1);
        Eigen::EigenSolver<Eigen::Matrix<double,2,2> > es(covariance);
	    Eigen::Matrix<double,2,1> Ev_one;
	    Eigen::Matrix<double,2,1> Ev_two;
	    Ev_one << es.eigenvectors()(0,0).real(),es.eigenvectors()(1,0).real();
	    Ev_one = Ev_one.normalized();
	    Ev_two << es.eigenvectors()(0,1).real(),es.eigenvectors()(1,1).real();
	    Ev_two = Ev_one.normalized();
	    double ev_one = es.eigenvalues()(0).real();
	    double ev_two = es.eigenvalues()(1).real();

	    // initiallize ellipse parameters
	    int number_of_points = 64;
	    double ellipse_a = number_of_stddev*sqrt(ev_one);
	    double ellipse_b = number_of_stddev*sqrt(ev_two);
	    double theta_offset = atan2(Ev_one(1),Ev_one(0));
	    Eigen::Matrix<double,2,2> rotation_matrix;
	    rotation_matrix << cos(theta_offset), -sin(theta_offset),
	    				   sin(theta_offset),  cos(theta_offset);

		// build ellipse and push points to marker
		Eigen::Matrix<double,2,1> base_point;
		Eigen::Matrix<double,2,1> transformed_point;
		Eigen::Matrix<double,2,1> offset_point;
		offset_point << marker_state(0),marker_state(1);
	    geometry_msgs::Point push_point;
	    push_point.z = 0;
	    for(int i = 0; i <= number_of_points; i++){
	    	base_point(0) = ellipse_a*cos(i*2*M_PI/number_of_points);
	    	base_point(1) = ellipse_b*sin(i*2*M_PI/number_of_points);
	    	transformed_point = rotation_matrix*base_point+offset_point;
	    	push_point.x = transformed_point(0);
	    	push_point.y = transformed_point(1);
	    	marker.points.push_back(push_point);
	    }
        ellipse_pub.publish(marker);
    }

    void updateArcStateVector(Eigen::Matrix<double, 3, 1> exact_marker_state, Eigen::Matrix<double, 3, 1> gps_marker_state, Eigen::Matrix<double,3,3> full_covariance, double number_of_stddev){

    	visualization_msgs::Marker marker;
    	marker.type = visualization_msgs::Marker::LINE_STRIP;
    	marker.header.frame_id = TF_FIXED_FRAME;
        marker.scale.x = THIN_LINE_WIDTH;
        marker.scale.y = THIN_LINE_WIDTH;
        marker.scale.z = THIN_LINE_WIDTH;
        marker.color.a = 1.0;
        marker.color.r = r;        
        marker.color.g = g;
        marker.color.b = b; 

	    // initiallize arc parameters
	    int number_of_points = 8;
	    double arc_radius = 0.75;
	    double arc_dist = number_of_stddev*sqrt(full_covariance(2,2));

	    // build ellipse and push points to marker
		Eigen::Matrix<double,2,1> base_point;
	    Eigen::Matrix<double,2,1> offset_point;
		offset_point << gps_marker_state(0),gps_marker_state(1); 
	    geometry_msgs::Point push_point;
	    push_point.x = offset_point(0);
	    push_point.y = offset_point(1);
	    push_point.z = 0;
	    marker.points.push_back(push_point);
	    for(int i = 0; i <= number_of_points; i++){
	    	push_point.x = offset_point(0)+arc_radius*cos(exact_marker_state(2)-arc_dist+(i*(2*arc_dist/number_of_points)));
	    	push_point.y = offset_point(1)+arc_radius*sin(exact_marker_state(2)-arc_dist+(i*(2*arc_dist/number_of_points)));
	    	marker.points.push_back(push_point);
	    }
	    push_point.x = offset_point(0);
	    push_point.y = offset_point(1);
	    marker.points.push_back(push_point);

        arc_pub.publish(marker);
    }
};

#endif