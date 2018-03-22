package Autonomy;


import android.util.Log;

import org.ejml.simple.SimpleMatrix;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import Autonomy.MyQuaternion;

/**
 * Created by meskupie on 08/03/18.
 */

public class Planner {

    private double TRANSLATION_RATE = 0.05;
    private double ROTATION_RATE = 0.1;
    private double TRANSLATION_EPSILON = 0.075;
    private double ROTATION_EPSILON = 0.15;

    private Transform pose_current = new Transform(new Vector3(0,0,0), new Quaternion(0,0,0,1));
    private Transform pose_initial = new Transform(new Vector3(0,0,0), new Quaternion(0,0,0,1));
    private Transform pose_reference = new Transform(new Vector3(0,0,0),new Quaternion(0,0,0,1));
    private double[] state_reference = new double[12];

    public Vector3 translation_initial_error = new Vector3(0,0,0);
    public Vector3 rotation_initial_error = new Vector3(0,0,0);
    public boolean ready_initial_translation = false;
    public boolean ready_initial_rotation = false;

    private boolean ready_planner = false;
    private boolean ready_pose = false;
    //private boolean ready_path = false;
    private boolean ready_initial_pose = false;
    private boolean ready_run_mode = false;
    private boolean ready_teleop_style = false;

    private int run_mode;
    private int teleop_style;


    public Planner(){}

    public boolean calculateReference(){
        //pose_reference = new Transform(new Vector3(0,0,0),new Quaternion(0,0,0,1));
        if(ready_pose) {
            Transform delta = pose_current.invert().multiply(pose_reference);
            Vector3 delta_pos = delta.getTranslation();
            MyQuaternion delta_rot = new MyQuaternion(delta.getRotationAndScale());
            state_reference = new double[12];
            state_reference[0] = delta_pos.getX();
            state_reference[2] = delta_pos.getY();
            state_reference[4] = delta_pos.getZ();
            state_reference[6] = delta_rot.getRoll();
            state_reference[8] = delta_rot.getPitch();
            state_reference[10] = delta_rot.getYaw();
            ready_pose = false;

            return true;
        }
        return false;
    }

    public void reset(){
        pose_reference = pose_initial;
    }

    public boolean isClose(){
        Transform delta = pose_current.invert().multiply(pose_reference);
        translation_initial_error = delta.getTranslation();
        ready_initial_translation = (Math.abs(translation_initial_error.getZ()) <= TRANSLATION_EPSILON);
        MyQuaternion euler_rotation_error = new MyQuaternion(delta.getRotationAndScale());
        rotation_initial_error = new Vector3(euler_rotation_error.getRoll(),euler_rotation_error.getPitch(),euler_rotation_error.getYaw());
        ready_initial_rotation = (Math.abs(rotation_initial_error.getX()) <= ROTATION_EPSILON) && (Math.abs(rotation_initial_error.getX()) <= ROTATION_EPSILON) && (Math.abs(rotation_initial_error.getX()) <= ROTATION_EPSILON);
        return ready_initial_translation && ready_initial_rotation;
    }

    private boolean myIsReady(){
        if(ready_run_mode){
            switch(run_mode){
                case 0:
                    break;
                case 1:
                    break;
                case 2:
                    return ready_initial_pose&&ready_teleop_style;
            }
        }
        return false;
    }

    // Mutator
    public boolean setPose(Transform _pose){
        pose_current = _pose;
        ready_pose = true;
        return ready_pose;
    }

    public boolean setTeleopStyle(int _mode){
        teleop_style = _mode;
        ready_teleop_style = true;
        return ready_teleop_style;
    }

    public boolean setJoyInput(SimpleMatrix joy_input,double period) {
        if (ready_teleop_style) {
            // joy range should be -1 to 1
            for (int i = 0; i < joy_input.numRows(); i++) {
                if (joy_input.get(i) > 1 || joy_input.get(i) < -1) {
                    Log.d("ROV_ERROR", "Planner: Joystick input range is out of bounds");
                    joy_input = new SimpleMatrix(6, 1);
                }
            }
            Vector3 delta_translation;
            Quaternion delta_rotation;
            Transform delta;
            switch (teleop_style) {
                case 0: // Inertial frame translation only, fixed rotation
                    delta_translation = new Vector3(joy_input.get(0) * TRANSLATION_RATE * period, joy_input.get(1) * TRANSLATION_RATE * period, joy_input.get(2) * TRANSLATION_RATE * period);
                    pose_reference = new Transform(pose_reference.getTranslation().add(delta_translation),pose_reference.getRotationAndScale());
                    break;
                case 1: // Body frame rotation only, fixed translation
                    // TODO: make this
                    break;
                case 2: // Inertial frame translation, body frame rotation
                    delta_translation = new Vector3(joy_input.get(0) * TRANSLATION_RATE * period, joy_input.get(1) * TRANSLATION_RATE * period, joy_input.get(2) * TRANSLATION_RATE * period);
                    pose_reference = new Transform(pose_reference.getTranslation().add(delta_translation),pose_reference.getRotationAndScale());
                    delta_rotation = MyQuaternion.createFromEuler(joy_input.get(3) * ROTATION_RATE * period, joy_input.get(4) * ROTATION_RATE * period, joy_input.get(5) * ROTATION_RATE * period);
                    delta = new Transform(new Vector3(0,0,0), delta_rotation);
                    pose_reference = pose_reference.multiply(delta);
                    break;
                case 3: // Body frame all axis
                    delta_translation = new Vector3(joy_input.get(0) * TRANSLATION_RATE * period, joy_input.get(1) * TRANSLATION_RATE * period, joy_input.get(2) * TRANSLATION_RATE * period);
                    delta_rotation = MyQuaternion.createFromEuler(joy_input.get(3) * ROTATION_RATE * period, joy_input.get(4) * ROTATION_RATE * period, joy_input.get(5) * ROTATION_RATE * period);
                    delta = new Transform(delta_translation, delta_rotation);
                    pose_reference = pose_reference.multiply(delta);
                    break;
            }
            return true;
        }
        return false;
    }

    // Parameters
    public boolean setRunMode(int _run_mode) {
        run_mode = _run_mode;
        ready_run_mode = true;
        ready_planner = myIsReady();
        return ready_run_mode;
    }

    public boolean setInitialPoseData(Transform _pose_initial){
        pose_initial = _pose_initial;
        ready_initial_pose = true;
        ready_planner = myIsReady();
        return ready_initial_pose;
    }

    public boolean setPath(){
        return false;
    }

    // Accessors
    public double[] getStateReference(){
        return state_reference;
    }
    public boolean isReady(){
        return ready_planner;
    }
}
