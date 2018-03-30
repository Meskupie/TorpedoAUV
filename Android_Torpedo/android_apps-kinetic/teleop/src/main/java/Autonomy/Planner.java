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
//    private double TRANSLATION_RATE = 0.2;
//    private double ROTATION_RATE = 0.25;
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
    private boolean ready_path = false;
    private boolean ready_initial_pose = false;
    private boolean ready_run_mode = false;
    private boolean ready_teleop_style = false;

    double yaw_heading = 0;

    private boolean in_auto;

    private int run_mode;
    private int teleop_style;

    private SimpleMatrix joystick = new SimpleMatrix(6,1);

    public Planner(){}

    public boolean calculateReference(){
        //pose_reference = new Transform(new Vector3(0,0,0),new Quaternion(0,0,0,1));
        //Vector3 temp_pos = pose_reference.getTranslation();
        //MyQuaternion temp_rot = new MyQuaternion(pose_reference.getRotationAndScale());
        //Log.d("ROV_STATE",String.format("X:%2.2f  Y:%2.2f  Z:%2.2f  R:%1.3f  P:%1.3f  W:%1.3f",temp_pos.getX(),temp_pos.getY(),temp_pos.getZ(),temp_rot.getRoll(),temp_rot.getPitch(),temp_rot.getYaw()));
        if(ready_pose) {
//            Vector3 temp_pos = pose_current.getTranslation();
//            MyQuaternion temp_rot = new MyQuaternion(pose_current.getRotationAndScale());
//            Log.d("DEBUG_MSG","Robot Pose, X:"+temp_pos.getX()+" Y:"+temp_pos.getY()+" Z:"+temp_pos.getZ()+" qX:"+temp_rot.getX()+" qY:"+temp_rot.getY()+" qZ:"+temp_rot.getZ()+" qW:"+temp_rot.getW());//+" Roll:"+temp_rot.getRoll()+" Pitch:"+temp_rot.getPitch()+" Yaw:"+temp_rot.getYaw());
//            Vector3 temp_ref_pos = pose_reference.getTranslation();
//            MyQuaternion temp_ref_rot = new MyQuaternion(pose_reference.getRotationAndScale());
//            Log.d("DEBUG_MSG","Robot Ref , X:"+temp_ref_pos.getX()+" Y:"+temp_ref_pos.getY()+" Z:"+temp_ref_pos.getZ()+" qX:"+temp_ref_rot.getX()+" qY:"+temp_ref_rot.getY()+" qZ:"+temp_ref_rot.getZ()+" qW:"+temp_ref_rot.getW());//" Roll:"+temp_ref_rot.getRoll()+" Pitch:"+temp_ref_rot.getPitch()+" Yaw:"+temp_ref_rot.getYaw());

//            Transform delta = pose_reference.invert().multiply(pose_current);
//            Vector3 delta_pos = delta.getTranslation();
//            MyQuaternion delta_rot = new MyQuaternion(delta.getRotationAndScale());
            state_reference = new double[12];

            if(in_auto){
                state_reference[1] = 2;
                state_reference[10] = (yaw_heading-(new MyQuaternion(pose_current.getRotationAndScale()).getYaw()))*5;
            } else {
                state_reference[1] = 2;
                state_reference[4] = joystick.get(2)*8;
                state_reference[3] = joystick.get(1)*0.5;
                yaw_heading = yaw_heading+(joystick.get(5)*0.05);
                state_reference[10] = (yaw_heading-(new MyQuaternion(pose_current.getRotationAndScale()).getYaw()))*5;
                if (joystick.get(0) < 0){
                    state_reference = new double[]{0,0,0,0,0,0,0,0,0,0,0,0};
                } else if(joystick.get(0) > 0){
                    state_reference[1] = state_reference[1]+joystick.get(0)*2;
                }
            }
            Log.d("ROV_CONTROL",""+state_reference[1]+" "+state_reference[4]+" "+state_reference[3]+" ");

            pose_reference = new Transform(new Vector3(0,0,0),pose_reference.getRotationAndScale());

//            if(teleop_style == 4){
//                state_reference[1] = delta_pos.getX()/0.05;
//                state_reference[3] = delta_pos.getY()/0.05;
//                state_reference[4] = delta_pos.getZ()/0.05;
//                state_reference[10] = delta_rot.getYaw();
//                pose_reference = new Transform(new Vector3(0,0,0),pose_reference.getRotationAndScale());
//            }else {
//                state_reference[0] = delta_pos.getX();
//                state_reference[2] = delta_pos.getY();
//                state_reference[4] = delta_pos.getZ();
//                state_reference[6] = delta_rot.getRoll();
//                state_reference[8] = delta_rot.getPitch();
//                state_reference[10] = delta_rot.getYaw();
//            }


            ready_pose = false;
            return true;
        }
        return false;
    }

    public void reset(){
        pose_reference = pose_initial;
    }

    public boolean isClose(){
//        Vector3 temp_pos = pose_current.getTranslation();
//        MyQuaternion temp_rot = new MyQuaternion(pose_current.getRotationAndScale());
//        Log.d("DEBUG_MSG","Robot Pose, X:"+temp_pos.getX()+" Y:"+temp_pos.getY()+" Z:"+temp_pos.getZ()+" qX:"+temp_rot.getX()+" qYPitch:"+temp_rot.getY()+" qZ:"+temp_rot.getZ()+" qW:"+temp_rot.getW());//+" Roll:"+temp_rot.getRoll()+" Pitch:"+temp_rot.getPitch()+" Yaw:"+temp_rot.getYaw());
//        Vector3 temp_ref_pos = pose_reference.getTranslation();
//        MyQuaternion temp_ref_rot = new MyQuaternion(pose_reference.getRotationAndScale());
//        Log.d("DEBUG_MSG","Robot Ref , X:"+temp_ref_pos.getX()+" Y:"+temp_ref_pos.getY()+" Z:"+temp_ref_pos.getZ()+" qX:"+temp_ref_rot.getX()+" qYPitch:"+temp_ref_rot.getY()+" qZ:"+temp_ref_rot.getZ()+" qW:"+temp_ref_rot.getW());//" Roll:"+temp_ref_rot.getRoll()+" Pitch:"+temp_ref_rot.getPitch()+" Yaw:"+temp_ref_rot.getYaw());

        Transform delta = pose_current.invert().multiply(pose_reference);
        translation_initial_error = delta.getTranslation();
        ready_initial_translation = (Math.abs(translation_initial_error.getZ()) <= TRANSLATION_EPSILON);
        MyQuaternion euler_rotation_error = new MyQuaternion(delta.getRotationAndScale());
        rotation_initial_error = new Vector3(euler_rotation_error.getRoll(),euler_rotation_error.getPitch(),euler_rotation_error.getYaw());
        ready_initial_rotation = (Math.abs(rotation_initial_error.getX()) <= ROTATION_EPSILON) && (Math.abs(rotation_initial_error.getX()) <= ROTATION_EPSILON) && (Math.abs(rotation_initial_error.getX()) <= ROTATION_EPSILON);
        //Log.d("DEBUG_MSG",""+pose_current.getTranslation().getZ()+"  "+Math.abs(translation_initial_error.getZ())+"  "+Math.abs(rotation_initial_error.getX())+"  "+Math.abs(rotation_initial_error.getY())+"  "+Math.abs(rotation_initial_error.getZ()));
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
    public void inAuto(boolean _in_auto){
        in_auto = _in_auto;
    }

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
            teleop_style = 4;
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
                case 4: // Body frame rate translation, inertial orientation and z
                    joystick = joy_input;
//                    delta_translation = new Vector3(joy_input.get(0), joy_input.get(1), joy_input.get(2));
//                    delta_rotation = MyQuaternion.createFromEuler(0, 0, joy_input.get(5) * ROTATION_RATE * period);
//                    delta = new Transform(delta_translation, delta_rotation);
//                    pose_reference = pose_reference.multiply(delta);
//                    break;

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
        pose_reference = pose_initial;
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
