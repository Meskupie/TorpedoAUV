package Autonomy;

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
    double[] input_thrust_max = new double[]{15,15,15,15,10,10};
    double[] input_thrust_min = new double[]{-15,-15,-15,-15,-10,-10};

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
            state_reference_mat = new SimpleMatrix(12,1,false,state_reference);
            input_thrust_mat = lqr_K_mat.mult(state_reference_mat);
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
            for (int j = 0; j < SIZE_STATES; j++) {
                lqr_K_mat.set(i, j, (double) data_K_arr.get(j + i * SIZE_STATES));
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
