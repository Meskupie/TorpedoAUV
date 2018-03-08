package com.github.rosjava.android_apps.teleop;

import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;


/**
 * Created by meskupie on 07/03/18.
 */

public class Localization {

    private Transform pose;
    private Transform transform_imu;

    public Localization(){

    }

//    public geometry_msgs.Transform updatePose(geometry_msgs.Quaternion imu_data){
//
//
//    }

    public boolean setInitialOrientation(geometry_msgs.Quaternion imu_data_msg){
        Quaternion imu_data = new Quaternion(imu_data_msg.getX(),imu_data_msg.getY(),imu_data_msg.getZ(),imu_data_msg.getW());
        //imu_data.
        //Quaternion transform_quaternion =
        //transform_imu = new Transform(new Vector3(0,0,0),transform_quaternion);
        return true;
    }

}
