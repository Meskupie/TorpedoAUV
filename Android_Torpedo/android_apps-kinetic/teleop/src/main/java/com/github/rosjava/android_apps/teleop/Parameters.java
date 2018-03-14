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
    ArrayList<Double> data_A;
    ArrayList<Double> data_B;
    ArrayList<Double> data_K;
    ArrayList<Double> data_map;

    // Other
    private String path = "res/raw/";
    private final int SIZE_A = 144;
    private final int SIZE_B = 72;
    private final int SIZE_K = 72;

    public Parameters() {
        // Initialize data
        data_A = new ArrayList<>(SIZE_A);
        data_B = new ArrayList<>(SIZE_B);
        data_K = new ArrayList<>(SIZE_K);
        data_map = new ArrayList<>();
    }

    public boolean updateDynamics(String _filename) {
        String filename = path + _filename;
        InputStream is = this.getClass().getClassLoader().getResourceAsStream(filename);
        BufferedReader reader = new BufferedReader(new InputStreamReader(is));
        data_A = new ArrayList<>(SIZE_A);
        data_B = new ArrayList<>(SIZE_B);
        data_K = new ArrayList<>(SIZE_K);
        String line;
        try {
            for(int i = 0; i < SIZE_A; i++) {
                line = reader.readLine();
                if(line == null){
                    Log.e("ROV_ERROR", "Matrix A file read error");
                    return false;}
                data_A.add(Double.parseDouble(line));
            }
            reader.readLine();
            for(int i = 0; i < SIZE_B; i++) {
                line = reader.readLine();
                if(line == null){
                    Log.e("ROV_ERROR", "Matrix B file read error");
                    return false;}
                data_B.add(Double.parseDouble(line));
            }
            reader.readLine();
            for(int i = 0; i < SIZE_K; i++) {
                line = reader.readLine();
                if(line == null){
                    Log.e("ROV_ERROR", "Matrix K file read error");
                    return false;}
                data_K.add(Double.parseDouble(line));
            }
            reader.close();
        } catch (Exception e) {
            Log.e("ROV_ERROR", "Catch error on dynamics file read");
            e.printStackTrace();
            return false;
        }
        return true;
    }

    public boolean updateMap(String _filename){
        String filename = path + _filename;
        InputStream is = this.getClass().getClassLoader().getResourceAsStream(filename);
        BufferedReader reader = new BufferedReader(new InputStreamReader(is));
        data_map = new ArrayList<>();
        String line;
        try {
            int count = Integer.parseInt(reader.readLine());
            double scale = Double.parseDouble(reader.readLine());
            for(int i = 0; i < count; i++){
                line = reader.readLine();
                data_map.add(Double.parseDouble(reader.readLine()));
                data_map.add(Double.parseDouble(reader.readLine()));
                data_map.add(Double.parseDouble(reader.readLine()));
                data_map.add((double)Integer.parseInt(reader.readLine()));
            }
            reader.close();
        } catch (Exception e) {
            Log.e("ROV_ERROR", "Catch error on map file read");
            e.printStackTrace();
            return false;
        }
        return true;
    }

    // Accessors
    public ArrayList getDataA(){return data_A;}
    public ArrayList getDataB(){return data_B;}
    public ArrayList getDataK(){return data_K;}
    public ArrayList getDataMap(){return data_map;}

}