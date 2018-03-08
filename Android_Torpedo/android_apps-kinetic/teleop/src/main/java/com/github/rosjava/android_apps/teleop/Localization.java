package com.github.rosjava.android_apps.teleop;

import android.util.Log;

import org.ejml.dense.block.VectorOps_DDRB;
import org.ejml.simple.SimpleMatrix;
import org.ros.message.Time;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import java.util.concurrent.ThreadLocalRandom;


/**
 * Created by meskupie on 07/03/18.
 */

public class Localization {

    // Camera targets class
    static class CameraTarget {
        private String type;
        private int id;

        private double altitude;
        private double azimuth;

        private double x;
        private double y;
        private double z;

        public CameraTarget(double _altitude, double _azimuth, int _id){
            type = "measurement";
            altitude = _altitude;
            azimuth = _azimuth;
            id = _id;
        }

        public CameraTarget(double _x, double _y, double _z, int _id){
            type = "map";
            x = _x;
            y = _y;
            z = _z;
            id = _id;
        }
    }

    // Constants
    final private int SIZE_STATES = 12;
    final private int SIZE_INPUTS = 6;

    // Initialization
    private boolean spread_particles = true;
    private boolean pose_locked = false;

    // Pose
    private double pose_fitness;
    private Transform pose = new Transform(new Vector3(0,0,0),new Quaternion(0,0,0,1));

    // IMU transform
    private boolean ready_initial_orientation = false;
    private int imu_data_count = 0;
    private double imu_initial_yaw;
    private Transform transform_imu = new Transform(new Vector3(0,0,0),new Quaternion(0,0,0,1));

    // Sensor data
    private boolean ready_thrust = false;
    private boolean ready_imu = false;
    private boolean ready_depth = false;
    private boolean ready_camera = false;

    private Time time_thrust;
    private Time time_imu;
    private Time time_depth;
    private Time time_camera;

    private SimpleMatrix data_thrust;
    private Quaternion data_imu;
    private double data_depth;
    private CameraTarget[] data_camera_targets;

    // Parameters
    private boolean ready_A = false;
    private boolean ready_B = false;
    private boolean ready_map = false;

    private SimpleMatrix param_A_mat = new SimpleMatrix(SIZE_STATES,SIZE_STATES);
    private SimpleMatrix param_B_mat = new SimpleMatrix(SIZE_STATES,SIZE_INPUTS);
    private CameraTarget[] param_map;

    public Localization(){}

    public Boolean attemptUpdate(Time call_time){
        // Check if parameters are ready
        boolean ready_parameters;
        if(ready_A&&ready_B&&ready_map){
            ready_parameters = true;
        }else{
            ready_parameters = false;
        }

        // Check what sensor data is ready
        boolean ready_sensors;
        if(ready_imu){
            ready_sensors = true;
        }else{
            ready_sensors = false;
        }

        if(ready_parameters&&ready_sensors){
            pose = transform_imu.multiply(new Transform(new Vector3(0,0,0),data_imu));
            ready_imu = false;
            // TODO: add other resets
            return true;
        }

        return false;
    }

    public boolean setInitialOrientation(Quaternion _imu_data){
        double ewma_rate =  0.05;
        double imu_yaw = new MyQuaternion(_imu_data).getYaw();
        if(imu_data_count == 0){
            imu_initial_yaw = imu_yaw;
            ready_initial_orientation = false;
        }else{
            imu_initial_yaw = ewma_rate*imu_yaw+(1-ewma_rate)*imu_initial_yaw;
            if(imu_data_count > 10){
                ready_initial_orientation = true;
            }else{ready_initial_orientation = false;}
        }
        transform_imu = new Transform(new Vector3(0,0,0),MyQuaternion.createFromEuler(180,0,imu_initial_yaw).invert());
        imu_data_count ++;
        return true;
    }

    public boolean resetInitialOrientation(){
        imu_initial_yaw = 0;
        imu_data_count = 0;
        ready_initial_orientation = false;
        return !ready_initial_orientation;
    }

    // Mutator
    public boolean setImuData(Quaternion _data_imu, Time _time_imu){
        data_imu = _data_imu;
        time_imu = _time_imu;
        ready_imu = true;
        return ready_imu;
    }

    public boolean setThrusterData(double[] _data_thrusters){
        // TODO: add thruster matrix
        ready_thrust = true;
        return ready_thrust;
    }

    public boolean setCameraTargets(double[] _data_camera_targets){
        // TODO: add camera targets array
        ready_camera = true;
        return ready_camera;
    }

    public boolean setMapData(double[] _map){
        // TODO: add map array
        ready_map = true;
        return ready_map;
    }

    public boolean setAData(SimpleMatrix A){
        param_A_mat = A;
        ready_A = true;
        return ready_A;
    }

    public boolean setBData(SimpleMatrix B){
        param_B_mat = B;
        ready_B = true;
        return ready_B;
    }

    // Accessors
    public Transform getPose(){
        return pose;
    }

    public double getFitness(){
        return pose_fitness;
    }

    public int[] getAShape(){
        return new int[] {SIZE_INPUTS,SIZE_INPUTS};
    }

    public int[] getBShape(){
        return new int[] {SIZE_STATES,SIZE_INPUTS};
    }



//    public void test(){
//        Log.d("DEBUG_MSG"," ");
//        Transform base = new Transform(new Vector3(0,0,0), MyQuaternion.createFromEuler(180.0/180.0*3.14159,0.0/180.0*3.14159,45.0/180.0*3.14159).invert());
//        Transform delta = new Transform(new Vector3(0,0,0), MyQuaternion.createFromEuler(-21.0/180.0*3.14159,15.0/180.0*3.14159,5.0/180.0*3.14159));
//        Transform imu = base.multiply(delta);
//        MyQuaternion imu_q = new MyQuaternion(imu.getRotationAndScale());
//        Log.d("DEBUG_MSG","IMU  w:"+imu_q.getW()+" x:"+imu_q.getX()+" y:"+imu_q.getY()+" z:"+imu_q.getZ());
//        Log.d("DEBUG_MSG","IMU  r:"+(imu_q.getRoll()*(180/3.14159))+" p:"+(imu_q.getPitch()*(180/3.14159))+" w:"+(imu_q.getYaw()*(180/3.14159)));
//
//        MyQuaternion attitude = new MyQuaternion(base.multiply(imu).getRotationAndScale());
//        Log.d("DEBUG_MSG","NEW  w:"+attitude.getW()+" x:"+attitude.getX()+" y:"+attitude.getY()+" z:"+attitude.getZ());
//        Log.d("DEBUG_MSG","r:"+(attitude.getRoll()*(180/3.14159))+" p:"+(attitude.getPitch()*(180/3.14159))+" w:"+(attitude.getYaw()*(180/3.14159)));
//    }
}
