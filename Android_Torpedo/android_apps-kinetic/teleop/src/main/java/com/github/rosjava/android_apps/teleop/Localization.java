package com.github.rosjava.android_apps.teleop;

import android.util.Log;

import org.ejml.dense.block.VectorOps_DDRB;
import org.ejml.simple.SimpleMatrix;
import org.ros.message.Time;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import java.util.concurrent.ThreadLocalRandom;

import geometry_msgs.Twist;


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

        public CameraTarget(double _altitude, double _azimuth, int _id) {
            type = "measurement";
            altitude = _altitude;
            azimuth = _azimuth;
            id = _id;
        }

        public CameraTarget(double _x, double _y, double _z, int _id) {
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
    private Transform pose = new Transform(new Vector3(0, 0, 0), new Quaternion(0, 0, 0, 1));
    private double pose_fitness = 0;
    private Twist pose_twist;

    // IMU transform
    private boolean ready_initial_pose = false;
    private int imu_data_count = 0;
    private double imu_initial_yaw;
    private Transform transform_imu = new Transform(new Vector3(0, 0, 0), new Quaternion(0, 0, 0, 1));

    // State machine
    private boolean ready_localization;
    private boolean ready_A = false;
    private boolean ready_B = false;
    private boolean ready_map = true; // TODO: change this to false

    private boolean ready_thrust = false;
    private boolean ready_imu = false;
    private boolean ready_depth = false;
    private boolean ready_camera = false;

    private Time time_thrust;
    private Time time_imu;
    private Time time_depth;
    private Time time_camera;

    // Data
    private SimpleMatrix data_thrust;
    private Quaternion data_imu;
    private double data_depth;
    private CameraTarget[] data_camera_targets;

    private SimpleMatrix param_A_mat = new SimpleMatrix(SIZE_STATES, SIZE_STATES);
    private SimpleMatrix param_B_mat = new SimpleMatrix(SIZE_STATES, SIZE_INPUTS);
    private CameraTarget[] param_map;

    public Localization() {}

    public Boolean attemptUpdate(Time call_time) {
        // Check what sensor data is ready
        boolean ready_sensors = (ready_imu&&true);
        // Check if we should run
        if (ready_localization && ready_sensors) {
            pose = transform_imu.multiply(new Transform(new Vector3(0, 0, 0), data_imu));
            ready_imu = false;
            // TODO: add other resets
            return true;
        }
        return false;
    }

    public boolean setInitialOrientation() {
        if(ready_imu) {
            double ewma_rate = 0.05;
            double imu_yaw = new MyQuaternion(data_imu).getYaw();
            if (imu_data_count == 0) {
                imu_initial_yaw = imu_yaw;
                ready_initial_pose = false;
            } else {
                imu_initial_yaw = ewma_rate * imu_yaw + (1 - ewma_rate) * imu_initial_yaw;
                if (imu_data_count > 20) {
                    ready_initial_pose = true;
                } else {
                    ready_initial_pose = false;
                }
            }
            transform_imu = new Transform(new Vector3(0, 0, 0), MyQuaternion.createFromEuler(180, 0, imu_initial_yaw).invert());
            imu_data_count++;
            ready_imu = false;
            return true;
        }
        return false;
    }

    public boolean resetInitialOrientation() {
        imu_initial_yaw = 0;
        imu_data_count = 0;
        ready_initial_pose = false;
        return !ready_initial_pose;
    }

    private boolean myIsReady(){return(ready_A && ready_B && ready_map);}

    // Mutator
    public boolean setImuData(Quaternion _data_imu, Time _time_imu) {
        data_imu = _data_imu;
        time_imu = _time_imu;
        ready_imu = true;
        return ready_imu;
    }

    public boolean setThrusterData(double[] _data_thrusters) {
        // TODO: add thruster matrix
        ready_thrust = true;
        return ready_thrust;
    }

    public boolean setCameraTargets(double[] _data_camera_targets) {
        // TODO: add camera targets array
        ready_camera = true;
        return ready_camera;
    }

    public boolean setMapData(double[] _map) {
        // TODO: add map array
        ready_map = true;
        ready_localization = myIsReady();
        return ready_map;
    }

    public boolean setAData(SimpleMatrix A) {
        param_A_mat = A;
        ready_A = true;
        ready_localization = myIsReady();
        return ready_A;
    }

    public boolean setBData(SimpleMatrix B) {
        param_B_mat = B;
        ready_B = true;
        ready_localization = myIsReady();
        return ready_B;
    }

    // Accessors
    public boolean isReady(){return ready_localization;}
    public boolean isLocked(){return ready_initial_pose;}
    public Transform getPose() {return pose;}
    public double getFitness() {return pose_fitness;}
    public Twist getTwist() {return pose_twist;}
    public int[] getAShape() {return new int[]{SIZE_INPUTS, SIZE_INPUTS};}
    public int[] getBShape() {return new int[]{SIZE_STATES, SIZE_INPUTS};}
}