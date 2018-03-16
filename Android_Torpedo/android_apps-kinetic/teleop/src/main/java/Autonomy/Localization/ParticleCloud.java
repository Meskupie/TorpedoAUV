package Autonomy.Localization;

import android.util.Log;

import org.ejml.data.Complex_F64;
import org.ejml.simple.SimpleEVD;
import org.ejml.simple.SimpleMatrix;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import Autonomy.MyQuaternion;

/**
 * Created by meskupie on 13/03/18.
 */

public class ParticleCloud {
    private int particles_size;
    public Particle[] particles;

    private double[] initial_stddev_array;
    private SimpleMatrix initial_stddev;
    private double[] update_stddev_array;
    private SimpleMatrix update_stddev;

    Random random = new Random();

    public ParticleCloud(int count){
        particles_size = count;
        particles = new Particle[particles_size];
        setInitialStddev(1.5, 0.5);
        setUpdateStddev(0.3,0.2);
    }

    // Group functions on all particles
    public void initializeParticles(Transform pose){

        for(int i = 0; i < particles_size; i++){
            particles[i].setPose(pose);
            particles[i].randomizeState(initial_stddev);
        }
    }

    public void propagateParticles(SimpleMatrix A, SimpleMatrix B, SimpleMatrix u){
        for(int i = 0; i < particles_size; i++){
            particles[i].propagateState(A,B,u,update_stddev);
        }
    }

    public void resampleParticles(){
        Particle[] particles_new = new Particle[particles_size];
        double fitness_max = 0;
        double fitness_cur;
        double[] fitnesses = new double[particles_size];
        for(int i = 0; i < particles_size; i++){
            fitness_cur = particles[i].getFitness();
            fitnesses[i] = fitness_cur;
            if(fitness_cur > fitness_max){
                fitness_max = fitness_cur;
            }
        }
        int index = 0;
        double beta = 0;
        double beta_delta;
        double beta_max = fitness_max*2;
        for(int i = 0; i < particles_size; i++){
            beta_delta = random.nextDouble()*beta_max;
            beta += beta_delta;
            while(beta > fitnesses[index]){
                beta -= fitnesses[index];
                index = (index + 1)%particles_size;
            }
            particles_new[i] = particles[index];
        }
        particles = particles_new;
    }

    public void correctParticles(double update_period){
        for(int i = 0; i < particles_size; i++){
            particles[i].correctPose(update_period);
        }
    }


    public Transform calculateAveragePose(){
        Vector3 average_point = new Vector3(0,0,0);
        SimpleMatrix Q = new SimpleMatrix(4,particles_size);
        // Build iteration through particles
        for(int i = 0; i < particles_size; i++){
            Transform pose = particles[i].getPose();
            average_point = average_point.add(pose.getTranslation());
            Quaternion q = pose.getRotationAndScale();
            Q.set(0,i,q.getX());
            Q.set(1,i,q.getY());
            Q.set(2,i,q.getZ());
            Q.set(3,i,q.getW());
        }
        // Find the average point
        average_point = average_point.scale(1/particles_size);

        // Use this fancy Eigen vector based average for quaternions
        SimpleEVD<SimpleMatrix> Q_EVD = Q.mult(Q.transpose()).eig();
        SimpleMatrix Q_Ev = new SimpleMatrix(4,1);
        Q_Ev.set(0,0,Q_EVD.getEigenvalue(0).getReal());
        Q_Ev.set(1,0,Q_EVD.getEigenvalue(1).getReal());
        Q_Ev.set(2,0,Q_EVD.getEigenvalue(2).getReal());
        Q_Ev.set(3,0,Q_EVD.getEigenvalue(3).getReal());
        double max = 0;
        int index = 0;
        for(int i = 0; i < 4; i++){
            double Ev = Q_EVD.getEigenvalue(0).getReal();
            if(Ev > max){
                max = Ev;
                index = i;
            }
        }
        SimpleMatrix q_mat = Q_EVD.getEigenVector(index);
        Quaternion average_orientation = new Quaternion(q_mat.get(0,0),q_mat.get(1,0),q_mat.get(2,0),q_mat.get(3,0));

        return new Transform(average_point,average_orientation);
    }

//    // Individual functions on single particle
//    public void transformParticle(Transform delta, int index){
//        if(index >= particles_size){
//            Log.e("ROV_ERROR", "Particle overrun on transformParticle");
//        }else{
//            particles[index].transformPose(delta);
//        }
//    }
//
//    public Transform getParticleTransform(int index){
//        if(index >= particles_size){
//            Log.e("ROV_ERROR", "Particle overrun on getParticleTransform");
//            return new Transform(new Vector3(0,0,0), new Quaternion(0,0,0,1));
//        }else{
//            return particles[index].getPose();
//        }
//    }
//
//    public void setFitness(int index, double fitness){
//        if(index >= particles_size){
//            Log.e("ROV_ERROR", "Particle overrun on setFitness");
//        }else{
//            particles[index].setFitness(fitness);
//        }
//    }

    // ParticleCloud Mutators
    public void setInitialStddev(double trans, double rot){
        initial_stddev_array = new double[]{trans,0,trans,0,trans,0,rot,0,rot,0,rot,0};
        initial_stddev = new SimpleMatrix(12,1,false,initial_stddev_array);
    }

    public void setUpdateStddev(double trans, double rot){
        update_stddev_array = new double[]{trans,0,trans,0,trans,0,rot,0,rot,0,rot,0};
        update_stddev = new SimpleMatrix(12,1,false,update_stddev_array);
    }

}
