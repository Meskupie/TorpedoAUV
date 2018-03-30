package Autonomy;

import android.util.Log;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;

/**
 * Created by meskupie on 05/03/18.
 */

public class Controller {

    private double DT = 0.05;

    private double[] input_thrust = new double[6];
    private double[] state_reference = new double[12];
    private double[] input_thrust_max = new double[]{8,8,8,8,4,4};
    private double[] input_thrust_min = new double[]{-8,-8,-8,-8,-4,-4};

    private final int SIZE_STATES = 12;
    private final int SIZE_INPUTS = 6;

    private SimpleMatrix lqi_K_mat = new SimpleMatrix(SIZE_INPUTS,SIZE_STATES+SIZE_STATES/2);
    private SimpleMatrix state_reference_mat = new SimpleMatrix(SIZE_STATES,1);
    private SimpleMatrix state_reference_integral_mat = new SimpleMatrix(SIZE_STATES/2,1);
    private SimpleMatrix input_thrust_mat = new SimpleMatrix(SIZE_INPUTS,1);

    private boolean ready_controller = false;
    private boolean ready_state_reference = false;

    public Controller() {}

    public boolean computeInputThrust (){
        state_reference[0] = 0;
        state_reference[1] = 0;//state_reference[1];
        state_reference[2] = 0;
        state_reference[3] = 0;//state_reference[3];
        state_reference[4] = 0;//state_reference[4];
        state_reference[5] = 0;//state_reference[5];
        state_reference[6] = state_reference[6];
        state_reference[7] = state_reference[7];
        state_reference[8] = state_reference[8];
        state_reference[9] = state_reference[9];
        state_reference[10] = state_reference[10];
        state_reference[11] = state_reference[11];

        if(ready_controller&&ready_state_reference){
            state_reference_integral_mat = state_reference_integral_mat.plus(new SimpleMatrix(6,1,false,new double[]{state_reference[0],state_reference[2],state_reference[4],state_reference[6],state_reference[8],state_reference[10]}).scale(DT));
            state_reference_mat = new SimpleMatrix(SIZE_STATES+SIZE_STATES/2,1);
            for(int i = 0; i < SIZE_STATES;i++){
                state_reference_mat.set(i,state_reference[i]);
            }
            for(int i = SIZE_STATES; i < state_reference_mat.numRows();i++){
                state_reference_mat.set(i,-state_reference_integral_mat.get(i-SIZE_STATES));
            }
            input_thrust_mat = lqi_K_mat.mult(state_reference_mat);
            for(int i = 0; i < SIZE_INPUTS; i++){
                input_thrust[i] = Math.min(Math.max(input_thrust_mat.get(i),input_thrust_min[i]),input_thrust_max[1]);
            }
            double[] ref = state_reference;
            double[] u = input_thrust;
            Log.d("ROV_CONTROL",String.format("Ref %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f| u %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f" , ref[0],ref[1],ref[2],ref[3],ref[4],ref[5],ref[6],ref[7],ref[8],ref[9],ref[10],ref[11],u[0],u[1],u[2],u[3],u[4],u[5]) );
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
            for (int j = 0; j < SIZE_STATES+SIZE_STATES/2; j++) {
                lqi_K_mat.set(i, j, (double) data_K_arr.get(j + i * SIZE_STATES+SIZE_STATES/2));
            }
        }
//        for(int i = 0; i < 6; i++){
//            Log.d("DEBUG_MSG","K matrix: "+lqr_K_mat.get(i,0)+" "+lqr_K_mat.get(i,1)+" "+lqr_K_mat.get(i,2)+" "+lqr_K_mat.get(i,3)+" "+lqr_K_mat.get(i,4)+" "+lqr_K_mat.get(i,5)+" "+lqr_K_mat.get(i,6)+" "+lqr_K_mat.get(i,7)+" "+lqr_K_mat.get(i,8)+" "+lqr_K_mat.get(i,9)+" "+lqr_K_mat.get(i,10)+" "+lqr_K_mat.get(i,11));
//        }
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
