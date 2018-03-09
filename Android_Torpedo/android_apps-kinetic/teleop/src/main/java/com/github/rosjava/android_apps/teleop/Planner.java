package com.github.rosjava.android_apps.teleop;


import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

/**
 * Created by meskupie on 08/03/18.
 */

public class Planner {

    private Transform pose_current = new Transform(new Vector3(0,0,0), new Quaternion(0,0,0,1));
    private double[] state_reference = new double[12];

    private boolean ready_planner = true;
    private boolean ready_pose = false;
    private boolean ready_path = false;

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

    // Mutator
    public boolean setPose(Transform _pose){
        pose_current = _pose;
        ready_pose = true;
        return true;
    }

    // Accessors
    public double[] getStateReference(){
        return state_reference;
    }

    public boolean isReady(){
        return ready_planner;
    }
}
