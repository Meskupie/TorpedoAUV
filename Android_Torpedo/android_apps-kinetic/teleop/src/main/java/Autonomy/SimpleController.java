package Autonomy;

import android.util.Log;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;

/**
 * Created by meskupie on 05/03/18.
 */

public class SimpleController {

    private double DT = 0.05;

    private double W_Kp = 1;
    private double D_Kp = 5;

    private double[] input_thrust = new double[6];
    private double[] state_reference = new double[12];
    private double[] input_thrust_max = new double[]{8,8,8,8,6,6};
    private double[] input_thrust_min = new double[]{1.7,1.7,1.7,1.7,-6,-6};

    private final int SIZE_STATES = 12;
    private final int SIZE_INPUTS = 6;

    private SimpleMatrix state_reference_mat = new SimpleMatrix(SIZE_STATES,1);
    private SimpleMatrix input_thrust_mat = new SimpleMatrix(SIZE_INPUTS,1);

    private boolean ready_controller = true;
    private boolean ready_state_reference = false;

    public SimpleController() {}

    public boolean computeInputThrust (){
        state_reference[0] = 0;
        state_reference[1] = state_reference[1]; // Forward speed
        state_reference[2] = 0;
        state_reference[3] = state_reference[3];
        state_reference[4] = state_reference[4]; // Depth sensor
        state_reference[5] = 0;
        state_reference[6] = 0;
        state_reference[7] = 0;
        state_reference[8] = 0;
        state_reference[9] = 0;
        state_reference[10] = state_reference[10]; // Yaw
        state_reference[11] = 0;

        if(ready_controller&&ready_state_reference){
            double X = state_reference[1];// Amount that is desired in the X direction
            double Y = state_reference[3];// Amount that is desired in the Y direction
            double W = state_reference[10];
            double Z = state_reference[4];

            Log.d("ROV_CONTROL",""+state_reference[1]);

            input_thrust[0] = X+Y+W;
            input_thrust[1] = X-Y-W;
            input_thrust[2] = X-Y+W;
            input_thrust[3] = X+Y-W;
            input_thrust[4] = Z;
            input_thrust[5] = Z;
            //input_thrust_mat = new SimpleMatrix(6,1,false,input_thrust);

            // Bound data
            for(int i = 0; i < SIZE_INPUTS; i++){
                input_thrust[i] = Math.min(Math.max(input_thrust[i],input_thrust_min[i]),input_thrust_max[1]);
            }
//            double[] ref = state_reference;
//            double[] u = input_thrust;
//            Log.d("ROV_CONTROL",String.format("Ref %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f| u %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f" , ref[0],ref[1],ref[2],ref[3],ref[4],ref[5],ref[6],ref[7],ref[8],ref[9],ref[10],ref[11],u[0],u[1],u[2],u[3],u[4],u[5]) );
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
        return false;
    }

    // Accessor
    public double[] getInputThrust(){
        return input_thrust;
    }
    public boolean isReady(){
        return ready_controller;
    }
}
