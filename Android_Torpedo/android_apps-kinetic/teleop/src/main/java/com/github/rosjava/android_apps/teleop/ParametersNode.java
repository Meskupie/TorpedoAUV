package com.github.rosjava.android_apps.teleop;

import android.util.Log;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.Duration;
import org.ros.message.MessageListener;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
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

import std_msgs.Int32;

/**
 * Created by meskupie on 06/03/18.
 */

public class ParametersNode extends AbstractNodeMain {
    private Parameters params = new Parameters();

    private Time time_current;
    private Time time_status_system;
    private Duration timeout_status_system= new Duration(0.1);

    private int status_system;
    private int status_parameters;

    // Update booleans
    private boolean update_dynamics = false;
    private boolean update_map = false;
    private boolean update_run_mode = false;


    public ParametersNode(){}

    public boolean setDynamics(String _filename){
        if(!params.updateDynamics(_filename)){
            Log.e("ROV_ERROR", "Parameters node: Failure reading dynamics");
            return false;
        }
        update_dynamics = true;
        return true;
    }

    public boolean setMap(String _filename){
        if(!params.updateMap(_filename)){
            Log.e("ROV_ERROR", "Parameters node: Failure reading map");
            return false;
        }
        update_map = true;
        return true;
    }

    public boolean setRunMode(int _mode){
        if(!params.updateRunMode(_mode)){
            Log.e("ROV_ERROR", "Parameters node: Failure updating run mode");
            return false;
        }
        update_run_mode = true;
        return true;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("ParametersNode");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        final Publisher<Int32> status_parameters_pub = connectedNode.newPublisher("status_parameters", Int32._TYPE);
        final Subscriber<Int32> status_system_sub = connectedNode.newSubscriber("status_system", Int32._TYPE);
        final ParameterTree param_tree = connectedNode.getParameterTree();

        connectedNode.executeCancellableLoop(new CancellableLoop() {
            Int32 status_parameters_msg = status_parameters_pub.newMessage();

            @Override
            protected void setup(){
                time_status_system = connectedNode.getCurrentTime();
            }

            @Override
            protected void loop() throws InterruptedException {
                // Check compliance
                if(status_system <= 2){
                    status_parameters |= 1;
                } else {status_parameters &= ~1;}

                // Check timeouts
                time_current = connectedNode.getCurrentTime();
                if(time_current.compareTo(time_status_system.add(timeout_status_system)) == 1){
                    if(status_system > 0){Log.e("ROV_ERROR", "Parameters node: Timeout on system state");}
                    status_parameters |= 2;
                } else { status_parameters &= ~2;}

                if(status_parameters == 0) {
                    // Update any parameters that have been modified
                    if (update_dynamics) {
                        param_tree.set("/dynamics_A", params.getDataA());
                        param_tree.set("/dynamics_B", params.getDataB());
                        param_tree.set("/controller_K", params.getDataK());
                        update_dynamics = false;
                    }
                    if (update_map){
                        param_tree.set("/planner_map", params.getDataMap());
                        update_map = false;
                    }
                    if (update_run_mode){
                        param_tree.set("/run_mode", params.getRunMode());
                        update_run_mode = false;
                    }

                }
                status_parameters_msg.setData(status_parameters);
                status_parameters_pub.publish(status_parameters_msg);
                Thread.sleep(1000);
            }
        });

        // System state callback
        status_system_sub.addMessageListener(new MessageListener<Int32>() {
            @Override
            public void onNewMessage(Int32 status_system_msg) {
                time_status_system = connectedNode.getCurrentTime();
                status_system = status_system_msg.getData();
            }
        });

    }
}
