package com.github.rosjava.android_apps.teleop;

import android.util.Log;

import org.ros.concurrent.CancellableLoop;
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
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.Scanner;

/**
 * Created by meskupie on 06/03/18.
 */

public class ParametersNode extends AbstractNodeMain {
    // update booleans
    private boolean update_dynamics;

    // parameters class
    private Parameters params;


    public ParametersNode(){
        // set updates to false
        update_dynamics = false;

        // parameter class object
        params = new Parameters();
    }

    public boolean setDynamics(String _filename){
        if(!params.updateDynamics(_filename)){
            Log.e("ROV_ERROR", "Unable to update dynamics");
            return false;
        }
        update_dynamics = true;
        return true;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("ParametersNode");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {

        final ParameterTree param_tree = connectedNode.getParameterTree();

        connectedNode.executeCancellableLoop(new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                // Test to see if data has been modified
                if(update_dynamics){
                    param_tree.set("/dynamics_A",params.getData_A());
                    param_tree.set("/dynamics_B",params.getData_B());
                    param_tree.set("/controller_K",params.getData_K());
                    update_dynamics = false;
                }

                Thread.sleep(1000);
            }
        });

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
