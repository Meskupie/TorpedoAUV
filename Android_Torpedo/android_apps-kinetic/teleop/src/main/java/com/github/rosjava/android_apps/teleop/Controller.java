package com.github.rosjava.android_apps.teleop;

import org.ejml.simple.SimpleMatrix;

import java.util.SimpleTimeZone;

/**
 * Created by meskupie on 05/03/18.
 */

public class Controller {
    private final int STATES = 12;
    private final int INPUTS = 6;

    private SimpleMatrix lqr_k_mat;
    private SimpleMatrix state_r_mat;
    private SimpleMatrix input_t_mat;

    private boolean ready;

    public Controller() {
        // Set variables to initial values
        lqr_k_mat = new SimpleMatrix(INPUTS,STATES);
        state_r_mat = new SimpleMatrix(STATES,1);
        input_t_mat = new SimpleMatrix(INPUTS,1);
        ready = false;
    }

    public boolean setGainMatrix(SimpleMatrix _lqr_k_mat) {
        // Check to make sure the input matrix is formatted correctly,
        if((_lqr_k_mat.numCols() == STATES)&&(_lqr_k_mat.numRows() == INPUTS)) {
            lqr_k_mat = _lqr_k_mat;
            ready = true;
            return true;
        } else {
            ready = false;
            return false;
        }
    }

    public SimpleMatrix computeInputThrust (SimpleMatrix _state_r_mat){
        if(ready){
            state_r_mat = _state_r_mat;
            input_t_mat = lqr_k_mat.mult(state_r_mat);
        } else {
            input_t_mat.zero();
        }
        return input_t_mat;
    }
}
