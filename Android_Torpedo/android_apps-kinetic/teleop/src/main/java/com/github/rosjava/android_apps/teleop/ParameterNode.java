package com.github.rosjava.android_apps.teleop;

import android.util.Log;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.yaml.snakeyaml.nodes.Node;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Scanner;

/**
 * Created by meskupie on 06/03/18.
 */

public class ParameterNode extends AbstractNodeMain {

    public List<Number> dynamics_a_data;
    public List<Number> dynamics_b_data;
    public List<Number> lqr_k_data;

    private String dynamics_path = "res/raw/";

    public ParameterNode(){
         lqr_k_data = new ArrayList<>();
    }

    public boolean setDynamics(String _filename){
        String filename = dynamics_path+_filename;
        InputStream is = this.getClass().getClassLoader().getResourceAsStream(filename);
        BufferedReader reader = new BufferedReader(new InputStreamReader(is));
        //Scanner scan;
        //DataInputStream input_stream = new DataInputStream(stream_handle);
        String line;
        double data;
        int count = 0;
        try {
            while ((line = reader.readLine()) != null) {
                data = Double.parseDouble(line);
                Log.d("DEBUG_MSG", line);//+count);
                count ++;
            }
        } catch(Exception e) {
            e.printStackTrace();
        }
        Log.d("DEBUG_MSG", ""+count);
//            scan = new Scanner(is);//.useDelimiter(",\\s*|\r\n|\n");
//            while(scan.hasNextLine()){
//                double data = Double.parseDouble(scan.nextLine());
//                Log.d("DEBUG_MSG", "Data: " + data);
//            }
//            while(input_stream.available()>0) {
//                Log.d("DEBUG_MSG","data:" + input_stream.);
//            }
//        } catch (java.util.InputMismatchException e) {
//            Log.d("DEBUG_MSG","Controller read error: " + e.getMessage());
//            return false;
//        }
        return true;
    }


    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("ParameterNode");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        ParameterTree params = connectedNode.getParameterTree();

        //params.set("/test_array", test);
    }

    @Override
    public void onShutdown(org.ros.node.Node node) {

    }

    @Override
    public void onShutdownComplete(org.ros.node.Node node) {

    }

    @Override
    public void onError(org.ros.node.Node node, Throwable throwable) {

    }
}
