package com.github.rosjava.android_apps.teleop;

import android.util.Log;

import org.ejml.simple.SimpleMatrix;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import std_msgs.Float64MultiArray;
import std_msgs.UInt8;

/**
 * Created by meskupie on 05/03/18.
 */

public class ControllerNode extends AbstractNodeMain{
    public Controller rov_controller = new Controller();
    private int system_state;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("ControllerNode");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        // Define pubs and subs
        final Publisher<Float64MultiArray> input_thrust_pub = connectedNode.newPublisher("input_thrust",Float64MultiArray._TYPE);
        final Publisher<UInt8> controller_state_pub = connectedNode.newPublisher("controller_state", UInt8._TYPE);
        final Publisher<std_msgs.Time> test_time_pub = connectedNode.newPublisher("controller_time", std_msgs.Time._TYPE);
        final Subscriber<Float64MultiArray> state_reference_sub = connectedNode.newSubscriber("state_reference", Float64MultiArray._TYPE);
        final Subscriber<UInt8> system_state_sub = connectedNode.newSubscriber("system_state", UInt8._TYPE);

        // Main cancelable loop
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            private Time temp_time;
            private std_msgs.Time temp_time_msg = test_time_pub.newMessage();

//            @Override
//            protected void setup() {
//
//            }

            @Override
            protected void loop() throws InterruptedException {
                temp_time = connectedNode.getCurrentTime();
                temp_time_msg.setData(temp_time);
                test_time_pub.publish(temp_time_msg);
                Thread.sleep(50);
            }
        });



        // Controller reference callback
        state_reference_sub.addMessageListener(new MessageListener<Float64MultiArray>() {
            @Override
            public void onNewMessage(Float64MultiArray state_r_msg) {

                // Declare publisher
                Float64MultiArray input_thrust_msg = input_thrust_pub.newMessage();

                // Set variables
                double[] state_r_arr = state_r_msg.getData();
                double[] input_t_arr = new double[6];
                SimpleMatrix state_r_mat = new SimpleMatrix(12,1);
                SimpleMatrix input_t_mat;

                // Pass reference into controller, pull out resulting input
                for(int i = 0; i < 12; i++){
                    state_r_mat.set(1,i,state_r_arr[i]);
                }
                input_t_mat = rov_controller.computeInputThrust(state_r_mat);
                for(int i = 0; i < 6; i++){
                    input_t_arr[i] = input_t_mat.get(i);
                }

                // Publish the motor commands
                input_thrust_msg.setData(input_t_arr);
                input_thrust_pub.publish(input_thrust_msg);
            }
        });

        // System state callback
        system_state_sub.addMessageListener(new MessageListener<UInt8>() {
            @Override
            public void onNewMessage(UInt8 system_state_msg) {
                system_state = system_state_msg.getData();
            }
        });
    }

    @Override
    public void onShutdown(Node node) {

    }

    @Override
    public void onShutdownComplete(Node node) {

    }

    @Override
    public void onError(Node node, Throwable throwable) {

    }
}
