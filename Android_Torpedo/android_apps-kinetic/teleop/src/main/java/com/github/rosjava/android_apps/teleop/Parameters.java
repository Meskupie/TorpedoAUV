package com.github.rosjava.android_apps.teleop;

import android.util.Log;

import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

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
    private ArrayList<Double> data_A;
    private ArrayList<Double> data_B;
    private ArrayList<Double> data_K;
    private ArrayList<Double> data_map;
    private int param_run_mode;
    private int param_teleop_style;
    private ArrayList<Double> param_initial_pose;

    // Other
    private String path = "res/raw/";
    private final int SIZE_A = 144;
    private final int SIZE_B = 72;
    private final int SIZE_K = 108;

    public Parameters() {
        // Initialize data
        data_A = new ArrayList<>(SIZE_A);
        data_B = new ArrayList<>(SIZE_B);
        data_K = new ArrayList<>(SIZE_K);
        data_map = new ArrayList<>();
        param_run_mode = 0;
        param_teleop_style = 0;
        param_initial_pose = new ArrayList<>(7);
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

    public boolean updateRunMode(int _mode){
        param_run_mode = _mode;
        return true;
    }

    public boolean updateTeleopStyle(int _mode){
        param_teleop_style = _mode;
        return true;
    }

    public boolean updateInitialPose(Transform _pose){
        param_initial_pose = new ArrayList<>(7);
        param_initial_pose.add(_pose.getTranslation().getX());
        param_initial_pose.add(_pose.getTranslation().getY());
        param_initial_pose.add(_pose.getTranslation().getZ());
        param_initial_pose.add(_pose.getRotationAndScale().getX());
        param_initial_pose.add(_pose.getRotationAndScale().getY());
        param_initial_pose.add(_pose.getRotationAndScale().getZ());
        param_initial_pose.add(_pose.getRotationAndScale().getW());
        return true;
    }

    // Accessors
    public ArrayList getDataA(){return data_A;}
    public ArrayList getDataB(){return data_B;}
    public ArrayList getDataK(){return data_K;}
    public ArrayList getDataMap(){return data_map;}
    public int getRunMode(){return param_run_mode;}
    public int getTeleopStyle(){return param_teleop_style;}
    public ArrayList getInitialPose(){return param_initial_pose;}
}