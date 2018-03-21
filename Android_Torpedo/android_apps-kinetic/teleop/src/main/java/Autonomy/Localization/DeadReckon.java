package Autonomy.Localization;

import android.util.Log;

import org.ejml.simple.SimpleMatrix;
import org.ros.message.Duration;
import org.ros.message.Time;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import Autonomy.MyQuaternion;

/**
 * Created by meskupie on 20/03/18.
 */

public class DeadReckon {

    // Constants
    final private int SIZE_STATES = 12;
    final private int SIZE_INPUTS = 6;
    final private double UPDATE_PERIOD = 0.05;

    // Pose
    private Transform pose_estimate;
    private Transform pose_inertial_cur = new Transform(new Vector3(0, 0, 0), new Quaternion(0, 0, 0, 1));
    private Transform pose_inertial_prev = pose_inertial_cur;
    private SimpleMatrix pose_body_cur = new SimpleMatrix(12,1);
    private SimpleMatrix pose_body_prev = new SimpleMatrix(12,1);
    Quaternion pose_imu_orientation;
    double pose_depth_z;

    // Sensor transforms
    private boolean ready_pose_lock = false;
    private int imu_data_count = 0;
    private double imu_initial_yaw;
    //private Transform transform_imu_fixed = new Transform(new Vector3(0, 0, 0), MyQuaternion.createFromEuler(0, 0,0)).invert();
    private Transform transform_imu = new Transform(new Vector3(0, 0, 0), new Quaternion(0, 0, 0, 1));
    private Transform transform_depth = new Transform(new Vector3(-0.1255,0.0125,-0.031), new Quaternion(0,0,0,1));

    // State machine
    private boolean ready_localization = false;
    private boolean ready_A = false;
    private boolean ready_B = false;

    private boolean ready_thrust = false;
    private boolean ready_imu = false;
    private boolean ready_depth = false;

    private Time time_last_update;
    private double time_update_period = 0.02;

    // Data
    private SimpleMatrix data_thrust = new SimpleMatrix(6,1);
    private Quaternion data_imu = new Quaternion(0,0,0,1);
    private double data_depth = 0;

    // Particle Filter
    private SimpleMatrix param_A_mat = new SimpleMatrix(SIZE_STATES, SIZE_STATES);
    private SimpleMatrix param_B_mat = new SimpleMatrix(SIZE_STATES, SIZE_INPUTS);


    public DeadReckon() {
    }

    public Boolean attemptInitialization(Time time_call){
        if(ready_localization){
            if(readySensors()) {
                // Average the IMU data to build an initial base orientation
                //Transform imu_data_transform = (transform_imu_fixed).multiply(new Transform(new Vector3(0,0,0),data_imu));
                double ewma_rate = 0.05;
                double imu_yaw = new MyQuaternion(data_imu).getYaw();
                if (imu_data_count == 0) {
                    imu_initial_yaw = imu_yaw;
                    ready_pose_lock = false;
                } else {
                    imu_initial_yaw = ewma_rate * imu_yaw + (1 - ewma_rate) * imu_initial_yaw;
                    if (imu_data_count > 20) {
                        ready_pose_lock = true;
                    } else {
                        ready_pose_lock = false;
                    }
                }
                transform_imu = new Transform(new Vector3(0, 0, 0), MyQuaternion.createFromEuler(0, 0, imu_initial_yaw)).invert();
//                transform_imu = (transform_imu_fixed).multiply(transform_imu_yaw);
//                MyQuaternion temp_rot = new MyQuaternion(transform_imu.getRotationAndScale());
//                Log.d("DEBUG_MSG","Transform IMU, Roll: "+temp_rot.getRoll()+" Pitch: "+temp_rot.getPitch()+" Yaw: "+temp_rot.getYaw());
                imu_data_count++;

                // Set the various poses
                pose_body_cur = new SimpleMatrix(12,1);
                pose_body_prev = pose_body_cur;
                // Resolve pose with sensor data
                this.resolvePose();

                return true;
            }
        }
        return false;
    }

    public Boolean attemptUpdate(Time time_call){

        if(ready_localization) {
            if(readySensors()) {

                // Propagate pose through motion model
                pose_body_cur = param_A_mat.mult(pose_body_prev).plus(param_B_mat.mult(data_thrust));
                Transform delta = new Transform(new Vector3(pose_body_cur.get(0),pose_body_cur.get(2),pose_body_cur.get(4)), MyQuaternion.createFromEuler(pose_body_cur.get(6),pose_body_cur.get(8),pose_body_cur.get(10)));
                pose_inertial_cur = pose_inertial_cur.multiply(delta);

                // Resolve pose with sensor data
                this.resolvePose();

                // Final estimate
                pose_estimate = pose_inertial_cur;
                MyQuaternion temp_rot = new MyQuaternion(pose_estimate.getRotationAndScale());
                Log.d("DEBUG_MSG","Robot Orientation, Roll: "+temp_rot.getRoll()+" Pitch: "+temp_rot.getPitch()+" Yaw: "+temp_rot.getYaw());

                // Reset the body pose
                Transform delta_final = pose_inertial_prev.invert().multiply(pose_inertial_cur);
                Vector3 trans = delta_final.getTranslation();
                MyQuaternion rot = new MyQuaternion(delta_final.getRotationAndScale());

                // Set body velocities
                pose_body_cur.set(1,0,trans.getX()/UPDATE_PERIOD);
                pose_body_cur.set(3,0,trans.getY()/UPDATE_PERIOD);
                pose_body_cur.set(5,0,trans.getZ()/UPDATE_PERIOD);
                pose_body_cur.set(7,0,rot.getRoll()/UPDATE_PERIOD);
                pose_body_cur.set(9,0,rot.getPitch()/UPDATE_PERIOD);
                pose_body_cur.set(11,0,rot.getYaw()/UPDATE_PERIOD);

                // Update old pose
                pose_body_prev = pose_body_cur;
                pose_inertial_prev = pose_inertial_cur;

                ready_imu = false;
                ready_depth = false;
                ready_thrust = false;
                return true;
            }
        }
        return false;
    }

    private void resolvePose(){
        // Transform sensor data
        Transform cur_inertial_orientation = transform_imu.multiply(new Transform(new Vector3(0,0,0),data_imu));
        pose_imu_orientation = cur_inertial_orientation.getRotationAndScale();
        Transform depth_transform = transform_depth.multiply(cur_inertial_orientation);
        pose_depth_z = data_depth-depth_transform.getTranslation().getZ();

        // Resolve orientation with IMU
        Vector3 pose_new_translation = new Vector3(pose_inertial_cur.getTranslation().getX(),pose_inertial_cur.getTranslation().getY(),pose_depth_z);
        Quaternion pose_new_orientation = pose_imu_orientation;
        pose_inertial_cur = new Transform(pose_new_translation,pose_new_orientation);
    }


    public boolean reset() {
        imu_initial_yaw = 0;
        imu_data_count = 0;
        ready_pose_lock = false;
        return !ready_pose_lock;
    }


    // ================= boring stuff ====================

    private boolean readySensors(){
        return ready_thrust && ready_imu && ready_depth;
    }

    // Mutator
    public boolean setImuData(Quaternion _data_imu) {
        data_imu = _data_imu;
        ready_imu = true;
        return ready_imu;
    }

    public boolean setDepthData(double _data_depth) {
        data_depth = _data_depth;
        ready_depth = true;
        return ready_depth;
    }

    public boolean setThrusterData(SimpleMatrix _data_thrust) {
        double ewma_rate = 0.1;
        data_thrust = _data_thrust.scale(ewma_rate).plus(data_thrust.scale((1-ewma_rate)));
        ready_thrust = true;
        return ready_thrust;
    }

    public boolean setCameraTargetsFront(CameraTarget[] _data_camera_targets) {
        return false;
    }

    public boolean setCameraTargetsRear(CameraTarget[] _data_camera_targets) {
        return false;
    }

    // ========  Parameter Mutators  ========
    private boolean myIsReady(){
        return(ready_A && ready_B);
    }
    public boolean setMapData(MapTarget[] _map) {
        return false;
    }
    public boolean setInitialPoseData(Transform _inital_pose) {
        return false;
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

    // ========  Accessors  ========
    public boolean isReady(){return ready_localization;}
    public boolean isLocked(){return ready_pose_lock;}
    public Transform getPose() {return pose_estimate;}
    public double getFitness() {return 0;}
    public Vector3 getLinearVelocity() {return new Vector3(0,0,0);}
    public Vector3 getAngularVelocity() {return new Vector3(0,0,0);}
    public int[] getAShape() {return new int[]{SIZE_STATES, SIZE_STATES};}
    public int[] getBShape() {return new int[]{SIZE_STATES, SIZE_INPUTS};}
}
