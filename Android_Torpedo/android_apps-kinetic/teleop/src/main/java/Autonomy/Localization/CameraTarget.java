package Autonomy.Localization;

import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import Autonomy.MyQuaternion;

/**
 * Created by meskupie on 13/03/18.
 */

// Camera targets class
public class CameraTarget {
    private int id;
    private Transform frame;

    private double altitude;
    private double azimuth;

//    private Vector3 point_target;
//    private Vector3 point_map;
//    private

    public CameraTarget(double _altitude, double _azimuth, int _id, Transform _frame) {
        altitude = _altitude;
        azimuth = _azimuth;
        id = _id;
        frame = _frame;
    }

    public CameraTarget(double _altitude, double _azimuth, int _id) {
        altitude = _altitude;
        azimuth = _azimuth;
        id = _id;
    }

    // Mutators
    public void setFrame(Transform _frame){
        frame = _frame;
    }

    // Accessors
    public Transform getTargetTransform(){
        Transform target_local = new Transform(new Vector3(0,0,0), MyQuaternion.createFromEuler(0,altitude,azimuth));
        return frame.multiply(target_local);
    }
    public int getTargetId(){return id;}
    public double getAltitude(){return altitude;}
    public double getAzimuth(){return azimuth;}
    public Transform getFrame(){return frame;}
}
