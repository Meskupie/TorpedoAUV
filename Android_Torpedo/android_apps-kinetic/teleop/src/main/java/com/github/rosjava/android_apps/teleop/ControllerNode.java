package com.github.rosjava.android_apps.teleop;

import android.util.Log;

import org.ejml.simple.SimpleMatrix;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.Duration;
import org.ros.message.MessageListener;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterListener;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.util.ArrayList;

import std_msgs.Float64MultiArray;
import std_msgs.Int16;
import std_msgs.Int32;
import std_msgs.Int8;
import std_msgs.UInt8;

public class ControllerNode extends AbstractNodeMain{
    private Controller rov_controller = new Controller();
    private int status_controller;
    private int system_state;

    private Time time_current;
    private Time time_state_r;
    private Time time_system_state;
    private Duration timeout_state_r = new Duration(0.1);
    private Duration timeout_system_state= new Duration(0.1);

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("ControllerNode");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        // Define ROS connections
        final Publisher<Int32> status_controller_pub = connectedNode.newPublisher("status_controller", Int32._TYPE);
        final Publisher<Float64MultiArray> input_thrust_pub = connectedNode.newPublisher("input_thrust",Float64MultiArray._TYPE);
        final Subscriber<Float64MultiArray> state_reference_sub = connectedNode.newSubscriber("state_reference", Float64MultiArray._TYPE);
        final Subscriber<Int32> system_state_sub = connectedNode.newSubscriber("system_state", Int32._TYPE);
        final ParameterTree param_tree = connectedNode.getParameterTree();

        // Main loop
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            Int32 status_controller_msg = status_controller_pub.newMessage();

            @Override protected void setup() {
                system_state = 0;
            }

            @Override
            protected void loop() throws InterruptedException {
                // Set variables
                time_current = connectedNode.getCurrentTime();

                // Check timeouts
                if(time_current.compareTo(time_state_r.add(timeout_state_r)) == 1){
                    Log.e("ROV_ERROR", "Controller node: Timeout on state reference");
                    status_controller |= 2;
                } else { status_controller &= ~2;}
                if(time_current.compareTo(time_system_state.add(timeout_system_state)) == 1){
                    Log.e("ROV_ERROR", "Controller node: Timeout on system state");
                    status_controller |= 2;
                } else { status_controller &= ~2;}

                // Check compliance
                if(system_state <= 3){
                    status_controller |= 1;
                } else {status_controller &= ~1;}

                // Publish status
                status_controller_msg.setData(status_controller);
                status_controller_pub.publish(status_controller_msg);
                Thread.sleep(20);
            }
        });

        // Controller reference callback
        state_reference_sub.addMessageListener(new MessageListener<Float64MultiArray>() {
            @Override
            public void onNewMessage(Float64MultiArray state_r_msg) {
                Float64MultiArray input_thrust_msg = input_thrust_pub.newMessage();
                time_state_r = connectedNode.getCurrentTime();
                // Calculate and publish the motor commands
                if(status_controller == 0){
                    input_thrust_msg.setData(rov_controller.computeInputThrust(state_r_msg.getData()));
                } else {
                    input_thrust_msg.setData(new double[6]);
                }
                input_thrust_pub.publish(input_thrust_msg);
            }
        });

        // System state callback
        system_state_sub.addMessageListener(new MessageListener<Int32>() {
            @Override
            public void onNewMessage(Int32 system_state_msg) {
                time_system_state = connectedNode.getCurrentTime();
                system_state = system_state_msg.getData();
            }
        });

        param_tree.addParameterListener("/controller_K", new ParameterListener() {
            @Override
            public void onNewValue(Object param_data) {rov_controller.setData_K((ArrayList<Number>) param_data);}
        });
    }
}
