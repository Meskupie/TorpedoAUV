package Autonomy.Localization;

import org.ejml.simple.SimpleMatrix;
import org.ros.message.Duration;
import org.ros.message.Time;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import Autonomy.MyQuaternion;


/**
 * Created by meskupie on 07/03/18.
 */

public class Localization {

    // Constants
    final private int SIZE_STATES = 12;
    final private int SIZE_INPUTS = 6;
    final private int PARTICLE_COUNT = 20;

    // Pose
    private Transform pose_estimate;
    private Transform pose_inertial_cur = new Transform(new Vector3(0, 0, 0), new Quaternion(0, 0, 0, 1));
    private Transform pose_inertial_prev = pose_inertial_cur;
    private Vector3 velocity_linear;
    private Vector3 velocity_angular;
    private double pose_fitness = 0;

    // Sensor transforms
    private boolean ready_pose_lock = false;
    private int imu_data_count = 0;
    private double imu_initial_yaw;
    private Transform transform_imu = new Transform(new Vector3(0, 0, 0), new Quaternion(0, 1, 0, 0));
    private double camera_transform_x = 0.240;
    private double camera_transform_z = 0.0043;
    private Transform camera_transform_front = new Transform(new Vector3( camera_transform_x,0,camera_transform_z),new Quaternion(0,0,0,1));
    private Transform camera_transform_rear  = new Transform(new Vector3(-camera_transform_x,0,camera_transform_z),new Quaternion(0,1,0,0));

    // State machine
    private boolean ready_localization = false;
    private boolean ready_A = false;
    private boolean ready_B = false;
    private boolean ready_map = false;
    private boolean ready_initial_pose = false;

    private boolean ready_thrust = false;
    private boolean ready_imu = false;
    private boolean ready_depth = false;
    private boolean ready_camera_front = false;
    private boolean ready_camera_rear = false;

    private Time time_last_update;
    private Duration timeout_camera_update = new Duration(0.025);
    private double time_update_period = 0.02;

    // Data
    private SimpleMatrix data_thrust;
    private Quaternion data_imu;
    private double data_depth;
    private CameraTarget[] data_camera_targets_front;
    private CameraTarget[] data_camera_targets_rear;

    // Particle Filter
    private ParticleCloud particles;
    private Transform param_initial_pose;
    private SimpleMatrix param_A_mat = new SimpleMatrix(SIZE_STATES, SIZE_STATES);
    private SimpleMatrix param_B_mat = new SimpleMatrix(SIZE_STATES, SIZE_INPUTS);

    // Data fitting
    private PoseFitting pose_fitting;
    private MapTarget[] param_map;
    private CameraTarget[] data_camera_targets_all;


    public Localization() {
        particles = new ParticleCloud(PARTICLE_COUNT);
        pose_fitting = new PoseFitting();
    }

    public Boolean attemptInitialization(Time time_call){
        if(ready_localization){
            if(readySensors() && readyCameras()) {
                particles.initializeParticles(param_initial_pose);
                return attemptUpdate(time_call);
            }
        }
        return false;
    }


    public Boolean attemptUpdate(Time time_call){
        if(ready_localization) {
            if((readySensors() && readyCameras()) || (readySensors() && checkTimeUpdate(time_call))) {
                time_last_update = time_call;
                // propagate particles through motion model
                particles.propagateParticles(param_A_mat,param_B_mat,data_thrust);

                if(readyCameras()){
                    // Build camera array
                    int front_length = data_camera_targets_front.length;
                    int rear_length = data_camera_targets_rear.length;
                    data_camera_targets_all = new CameraTarget[front_length+rear_length];
                    for(int i = 0; i < front_length; i++){
                        data_camera_targets_all[i] = data_camera_targets_front[i];}
                    for(int i = front_length; i < front_length+rear_length; i++){
                        data_camera_targets_all[i] = data_camera_targets_front[i];}
                    pose_fitting.setCameraTargets(data_camera_targets_all);

                    // Iterate through particles and run transforms
                    for(int i = 0; i < PARTICLE_COUNT; i++){
                        Transform particle_pose = particles.particles[i].getPose();
                        pose_fitting.fitTransform(particles.particles[i],2);
                        particles.particles[i].transformPose(pose_fitting.getPoseTransform());
                        particles.particles[i].setFitness(pose_fitting.getFitness());
                    }
                    ready_camera_front = false;
                    ready_camera_rear = false;
                }else{
                    // Iterate through particles and run transforms
                    for(int i = 0; i < PARTICLE_COUNT; i++){
                        pose_fitting.fitTransform(particles.particles[i],0);
                        particles.particles[i].transformPose(pose_fitting.getPoseTransform());
                        particles.particles[i].setFitness(pose_fitting.getFitness());
                    }
                }
                // resample particles and correct their velocities
                particles.resampleParticles();
                particles.correctParticles(time_update_period);

                // Calculate average particle pose and build the final estimate
                pose_inertial_cur = particles.calculateAveragePose();
                pose_estimate = pose_inertial_cur;

                // Correct the pose twists and extrapolate
                this.calculateVelocity(time_update_period); //update twist and prev state
                this.extrapolatePose(0.03);

                ready_imu = false;
                ready_depth = false;
                ready_thrust = false;

                return true;
            }
        }
        return false;
    }

    private void calculateVelocity(double update_period){
        Transform delta = pose_inertial_prev.invert().multiply(pose_inertial_cur);
        Vector3 trans = delta.getTranslation();
        MyQuaternion rot = new MyQuaternion(delta.getRotationAndScale());

        // Set body velocities
        velocity_linear = new Vector3(trans.getX()/update_period,trans.getY()/update_period,trans.getZ()/update_period);
        velocity_angular = new Vector3(rot.getRoll()/update_period,rot.getPitch()/update_period,rot.getYaw()/update_period);

        // Update old pose
        pose_inertial_prev = pose_inertial_cur;
    }

    private void extrapolatePose(double forward_time){
        if(forward_time > 0.001){
            MyQuaternion delta_rot = MyQuaternion.createFromEuler(velocity_angular.getX()*forward_time,velocity_angular.getZ()*forward_time,velocity_angular.getZ()*forward_time);
            Vector3 delta_trans = new Vector3(velocity_linear.getX()*forward_time,velocity_linear.getY()*forward_time,velocity_linear.getZ()*forward_time);
            Transform delta = new Transform(delta_trans,delta_rot);
            pose_estimate = pose_inertial_cur.multiply(delta);
        }
        pose_estimate = pose_inertial_cur;
    }

    private boolean checkTimeUpdate(Time time_call){
        if(time_call.compareTo(time_last_update.add(timeout_camera_update)) == 1){
            return true;
        }
        return false;
    }


    // ================= boring stuff ====================

    private boolean readySensors(){
        return ready_thrust && ready_imu;// && ready_depth;
    }
    private boolean readyCameras(){
        return ready_camera_front && ready_camera_rear;
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
        data_thrust = _data_thrust;
        ready_thrust = true;
        return ready_thrust;
    }

    public boolean setCameraTargetsFront(CameraTarget[] _data_camera_targets) {
        data_camera_targets_front = _data_camera_targets;
        for(int i = 0; i < data_camera_targets_front.length; i++){
            data_camera_targets_front[i].setFrame(camera_transform_front);}
        ready_camera_front = true;
        return ready_camera_front;
    }

    public boolean setCameraTargetsRear(CameraTarget[] _data_camera_targets) {
        data_camera_targets_rear = _data_camera_targets;
        for(int i = 0; i < data_camera_targets_rear.length; i++){
            data_camera_targets_rear[i].setFrame(camera_transform_rear);}
        ready_camera_rear = true;
        return ready_camera_rear;
    }


    // ========  Parameter Mutators  ========
    private boolean myIsReady(){
        return(ready_A && ready_B && ready_map && ready_initial_pose);
    }
    public boolean setMapData(MapTarget[] _map) {
        pose_fitting.setMap(_map);
        ready_map = true;
        ready_localization = myIsReady();
        return ready_map;
    }
    public boolean setInitialPoseData(Transform _inital_pose) {
        param_initial_pose = _inital_pose;
        ready_initial_pose = true;
        ready_localization = myIsReady();
        return ready_initial_pose;
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
    public double getFitness() {return pose_fitness;}
    public Vector3 getLinearVelocity() {return velocity_linear;}
    public Vector3 getAngularVelocity() {return velocity_angular;}
    public int[] getAShape() {return new int[]{SIZE_INPUTS, SIZE_INPUTS};}
    public int[] getBShape() {return new int[]{SIZE_STATES, SIZE_INPUTS};}

    // ========= deprecated =========
    public Boolean attemptUpdateIMU(Time call_time) {
        // Check what sensor data is ready
        boolean ready_sensors = (ready_imu&&true);
        // Check if we should run
        if (ready_localization && ready_sensors) {
            pose_inertial_cur = transform_imu.multiply(new Transform(new Vector3(0, 0, 0), data_imu));
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
}