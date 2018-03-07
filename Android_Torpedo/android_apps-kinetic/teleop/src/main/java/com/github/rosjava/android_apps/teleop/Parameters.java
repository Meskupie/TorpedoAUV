package com.github.rosjava.android_apps.teleop;

import android.util.Log;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Created by meskupie on 2018-03-06.
 */

public class Parameters {
    // Declare data
    ArrayList<Number> data_A;
    ArrayList<Number> data_B;
    ArrayList<Number> data_K;

    // Other
    private String dynamics_path = "res/raw/";
    private final int SIZE_A = 144;
    private final int SIZE_B = 72;
    private final int SIZE_K = 72;

    public Parameters() {
        // Initialize data
        data_A = new ArrayList<>(SIZE_A);
        data_B = new ArrayList<>(SIZE_B);
        data_K = new ArrayList<>(SIZE_K);
    }

    public boolean updateDynamics(String _filename) {
        String filename = dynamics_path + _filename;
        InputStream is = this.getClass().getClassLoader().getResourceAsStream(filename);
        BufferedReader reader = new BufferedReader(new InputStreamReader(is));
        String line;
        int count;
        try {
            count = 0;
            while ((line = reader.readLine()) != null) {
                data_A.set(count,Double.parseDouble(line));
                count++;
            }
            if (count + 1 != SIZE_A) {
                Log.e("ROV_ERROR", "Matrix A file read error");
                return false;
            }
            count = 0;
            while ((line = reader.readLine()) != null) {
                data_B.set(count,Double.parseDouble(line));
                count++;
            }
            if (count + 1 != SIZE_B) {
                Log.e("ROV_ERROR", "Matrix B file read error");
                return false;
            }
            count = 0;
            while ((line = reader.readLine()) != null) {
                data_K.set(count,Double.parseDouble(line));
                count++;
            }
            if (count + 1 != SIZE_K) {
                Log.e("ROV_ERROR", "Matrix K file read error");
                return false;
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        return true;
    }

    // Accessors
    public ArrayList getData_A(){return data_A;}
    public ArrayList getData_B(){return data_B;}
    public ArrayList getData_K(){return data_K;}

}