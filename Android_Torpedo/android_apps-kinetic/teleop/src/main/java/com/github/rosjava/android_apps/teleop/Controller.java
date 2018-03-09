package com.github.rosjava.android_apps.teleop;

import android.util.Log;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.SimpleTimeZone;

import tf2_msgs.LookupTransformGoal;

/**
 * Created by meskupie on 05/03/18.
 */

public class Controller {

    double[] input_thrust = new double[6];
    double[] state_reference = new double[12];

    private final int SIZE_STATES = 12;
    private final int SIZE_INPUTS = 6;

    private SimpleMatrix lqr_K_mat = new SimpleMatrix(SIZE_INPUTS,SIZE_STATES);
    private SimpleMatrix state_reference_mat = new SimpleMatrix(SIZE_STATES,1);
    private SimpleMatrix input_thrust_mat = new SimpleMatrix(SIZE_INPUTS,1);

    private boolean ready_controller = false;
    private boolean ready_state_reference = false;

    public Controller() {}

    public boolean computeInputThrust (){
        if(ready_controller&&ready_state_reference){
            for(int i = 0; i < SIZE_STATES; i++){
                state_reference_mat.set(1,i,state_reference[i]);
            }
            input_thrust_mat = lqr_K_mat.mult(state_reference_mat);
            for(int i = 0; i < SIZE_INPUTS; i++){
                input_thrust[i] = input_thrust_mat.get(i);
            }
            ready_state_reference = false;
            return true;
        }
        return false;
    }

    // Mutator
    public boolean setStateReference(double[] _state_reference_arr){
        ready_state_reference = true;
        state_reference = _state_reference_arr;
        return true;
    }

    public boolean setData_K (ArrayList<Number> data_K_arr){
        if(data_K_arr.size() != SIZE_STATES*SIZE_INPUTS){
            Log.e("ROV_ERROR","Matrix K set controller");
            return false;
        }
        for(int i = 0; i < SIZE_INPUTS; i++) {
            for(int j = 0; j < SIZE_STATES; j++) {
                lqr_K_mat.set(i,j,(double)data_K_arr.get(j+i*SIZE_STATES));
            }
        }
        ready_controller = true;
        return true;
    }

    // Accessor
    public double[] getInputThrust(){
        return input_thrust;
    }

    public boolean isReady(){
        return ready_controller;
    }
}
