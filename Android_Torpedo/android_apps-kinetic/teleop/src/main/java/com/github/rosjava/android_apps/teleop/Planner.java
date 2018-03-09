package com.github.rosjava.android_apps.teleop;


import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

/**
 * Created by meskupie on 08/03/18.
 */

public class Planner {

    private Transform pose = new Transform(new Vector3(0,0,0), new Quaternion(0,0,0,1));

    public Planner(){}



}
