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

    private double TRANSLATION_RATE = 1;
    private double ROTATION_RATE = 0.5;

    private Transform pose_current = new Transform(new Vector3(0,0,0), new Quaternion(0,0,0,1));
    private Transform pose_initial = new Transform(new Vector3(0,0,0), new Quaternion(0,0,0,1));
    private Transform pose_reference = new Transform(new Vector3(0,0,0),new Quaternion(0,0,0,1));
    private double[] state_reference = new double[12];

    private boolean ready_planner = false;
    private boolean ready_pose = false;
    //private boolean ready_path = false;
    private boolean ready_initial_pose = false;
    private boolean ready_run_mode = false;

    private int run_mode;


    public Planner(){}

    public boolean calculateReference(){
        if(ready_pose) {
            Vector3 position = pose_current.getTranslation();
            MyQuaternion attitude = new MyQuaternion(pose_current.getRotationAndScale());
            state_reference = new double[12];
            state_reference[0] = position.getX();
            state_reference[2] = position.getY();
            state_reference[4] = position.getZ();
            state_reference[6] = attitude.getRoll();
            state_reference[8] = attitude.getPitch();
            state_reference[10] = attitude.getYaw();
            ready_pose = false;

            return true;
        }
        return false;
    }

    public void reset(){
        pose_reference = pose_initial;
    }

    private boolean myIsReady(){
        if(ready_run_mode){
            switch(run_mode){
                case 0:
                    break;
                case 1:
                    break;
                case 2:
                    return ready_initial_pose;
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

    public boolean setJoyInput(SimpleMatrix joy_input,double period){
        // joy range should be -1 to 1
        for(int i = 0; i < joy_input.numRows(); i++) {
            if (joy_input.get(i) > 1 || joy_input.get(i) < -1) {
                Log.d("ROV_ERROR", "Planner: Joystick input range is out of bounds");
                joy_input = new SimpleMatrix(6,1);
            }
        }
        Vector3 delta_translation = new Vector3(joy_input.get(0)*TRANSLATION_RATE*period,joy_input.get(1)*TRANSLATION_RATE*period,joy_input.get(2)*TRANSLATION_RATE*period);
        Quaternion delta_rotation = MyQuaternion.createFromEuler(joy_input.get(3)*ROTATION_RATE*period,joy_input.get(4)*ROTATION_RATE*period,joy_input.get(5)*ROTATION_RATE*period);
        Transform delta = new Transform(delta_translation,delta_rotation);
        pose_reference = pose_reference.multiply(delta);
        return true;
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
