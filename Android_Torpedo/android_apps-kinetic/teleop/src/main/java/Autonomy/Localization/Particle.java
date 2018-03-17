package Autonomy.Localization;

import org.ejml.simple.SimpleMatrix;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import java.util.Random;

import Autonomy.MyQuaternion;

/**
 * Created by meskupie on 13/03/18.
 */

// Particle Class
public class Particle{
    Random random = new Random();
    private double fitness = 100000;
    private Transform pose_inertial_cur;
    private Transform pose_inertial_prev;
    private SimpleMatrix pose_body_cur = new SimpleMatrix(12, 1);
    private SimpleMatrix pose_body_prev = new SimpleMatrix(12, 1);

    public Particle(){
        pose_inertial_cur = new Transform(new Vector3(0,0,0),new Quaternion(0,0,0,1));
        pose_inertial_prev = pose_inertial_cur;
    }

    public void propagateState(SimpleMatrix A, SimpleMatrix B, SimpleMatrix u, SimpleMatrix stdev){
        pose_body_cur = A.mult(pose_body_prev).plus(B.mult(u));
        this.randomizeState(stdev);
        Transform delta = new Transform(new Vector3(pose_body_cur.get(0),pose_body_cur.get(2),pose_body_cur.get(4)), MyQuaternion.createFromEuler(pose_body_cur.get(6),pose_body_cur.get(8),pose_body_cur.get(10)));
        this.transformPose(delta);
    }

    public void transformPose(Transform delta){
        pose_inertial_cur = pose_inertial_cur.multiply(delta);
    }

    public void randomizeState(SimpleMatrix stddev){

        SimpleMatrix noise = new SimpleMatrix(12,1);
        for(int i = 0; i<12; i++){
            noise.set(i,0,random.nextGaussian()*stddev.get(i));
        }
        pose_body_cur = pose_body_cur.plus(noise);
    }

    public void correctPose(double update_period){
        Transform delta = pose_inertial_prev.invert().multiply(pose_inertial_cur);
        Vector3 trans = delta.getTranslation();
        MyQuaternion rot = new MyQuaternion(delta.getRotationAndScale());

        // Set body velocities
        pose_body_cur.set(1,0,trans.getX()/update_period);
        pose_body_cur.set(3,0,trans.getY()/update_period);
        pose_body_cur.set(5,0,trans.getZ()/update_period);
        pose_body_cur.set(7,0,rot.getRoll()/update_period);
        pose_body_cur.set(9,0,rot.getPitch()/update_period);
        pose_body_cur.set(11,0,rot.getYaw()/update_period);

        // Update old pose
        pose_body_prev = pose_body_cur;
        pose_inertial_prev = pose_inertial_cur;
    }

    // Mutators
    public void setPose(Transform _pose){
        pose_inertial_cur = _pose;
        pose_inertial_prev = pose_inertial_cur;
    }

    public void setFitness(double _fitness){
        fitness = _fitness;
    }

    // Accessors
    public Transform getPose(){return pose_inertial_cur;}
    public double getFitness(){return fitness;}
}
