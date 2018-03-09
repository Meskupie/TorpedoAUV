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
    private final int SIZE_STATES = 12;
    private final int SIZE_INPUTS = 6;

    private SimpleMatrix lqr_K_mat;
    private SimpleMatrix state_reference_mat;
    private SimpleMatrix input_thrust_mat;

    private boolean ready;

    public Controller() {
        // Set variables to initial values
        lqr_K_mat = new SimpleMatrix(SIZE_INPUTS,SIZE_STATES);
        state_reference_mat = new SimpleMatrix(SIZE_STATES,1);
        input_thrust_mat = new SimpleMatrix(SIZE_INPUTS,1);
        ready = false;
    }

    public double[] computeInputThrust (double[] _state_r_arr){
        if(ready){
            double[] state_r_arr = _state_r_arr;
            double[] input_t_arr = new double[6];
            SimpleMatrix state_r_mat = new SimpleMatrix(12,1);
            SimpleMatrix input_t_mat;

            for(int i = 0; i < 12; i++){
                state_r_mat.set(1,i,state_r_arr[i]);
            }
            input_t_mat = lqr_K_mat.mult(state_r_mat);
            for(int i = 0; i < 6; i++){
                input_t_arr[i] = input_t_mat.get(i);
            }
            return input_t_arr;
        } else {
            // return empty input
            return new double[6];
        }
    }

    // Mutator
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
        ready = true;
        return true;
    }
}
