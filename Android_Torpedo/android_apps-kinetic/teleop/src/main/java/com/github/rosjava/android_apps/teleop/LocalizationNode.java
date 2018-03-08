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
import org.ros.node.parameter.ParameterListener;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.rosjava_geometry.Quaternion;

import java.util.ArrayList;

import sensor_msgs.PointCloud;
import std_msgs.Float64MultiArray;
import std_msgs.Int32;

public class LocalizationNode extends AbstractNodeMain{
    private Localization rov_localization = new Localization();

    private int system_state;

    private Time time_system_state;
    private Time time_embedded_imu;
    private Time time_embedded_thrust;
    private Time time_camera_targets;

    private Duration timeout_system_state;
    private Duration timeout_embedded_imu;
    private Duration timeout_embedded_thrust;
    private Duration timeout_camera_targets;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("LocalizationNode");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        final Publisher<geometry_msgs.Transform> state_x_pub = connectedNode.newPublisher("state_x", geometry_msgs.Transform._TYPE);

        // Subscribers
        final Subscriber<Int32> system_state_sub = connectedNode.newSubscriber("system_state", Int32._TYPE);
        final Subscriber<geometry_msgs.Quaternion> embedded_imu_sub = connectedNode.newSubscriber("embedded_imu", geometry_msgs.Quaternion._TYPE);
        final Subscriber<Float64MultiArray> embedded_thrust_sub = connectedNode.newSubscriber("embedded_thrust", Float64MultiArray._TYPE);
        final Subscriber<PointCloud> camera_targets_sub = connectedNode.newSubscriber("camera_targets", PointCloud._TYPE);
        final ParameterTree param_tree = connectedNode.getParameterTree();


        connectedNode.executeCancellableLoop(new CancellableLoop() {
            @Override
            protected void setup(){

            }

            @Override
            protected void loop() throws InterruptedException {

                rov_localization.attemptUpdate(connectedNode.getCurrentTime());


                Thread.sleep(10);
            }
        });

        // System state callback
        system_state_sub.addMessageListener(new MessageListener<Int32>() {
            @Override
            public void onNewMessage(Int32 system_state_msg) {
                time_system_state = connectedNode.getCurrentTime();
                system_state = system_state_msg.getData();
                rov_localization.attemptUpdate(connectedNode.getCurrentTime());
            }
        });

        embedded_imu_sub.addMessageListener(new MessageListener<geometry_msgs.Quaternion>() {
            @Override
            public void onNewMessage(geometry_msgs.Quaternion imu_data) {
                time_embedded_imu = connectedNode.getCurrentTime();
                rov_localization.setImuData(new Quaternion(imu_data.getX(),imu_data.getY(),imu_data.getZ(),imu_data.getW()),time_embedded_imu);
                rov_localization.attemptUpdate(connectedNode.getCurrentTime());
            }
        });

        embedded_thrust_sub.addMessageListener(new MessageListener<Float64MultiArray>() {
            @Override
            public void onNewMessage(Float64MultiArray embedded_thrust_msg) {
                time_embedded_thrust = connectedNode.getCurrentTime();
                rov_localization.setThrusterData(embedded_thrust_msg.getData());
                rov_localization.attemptUpdate(connectedNode.getCurrentTime());
            }
        });

        camera_targets_sub.addMessageListener(new MessageListener<PointCloud>() {
            @Override
            public void onNewMessage(PointCloud pointCloud) {
                time_camera_targets = connectedNode.getCurrentTime();

                //rov_localization.setCameraTargets();
            }
        });

        param_tree.addParameterListener("/dynamics_A", new ParameterListener() {
            @Override
            public void onNewValue(Object A_data_msg) {
                ArrayList<Number> param_data = (ArrayList<Number>) A_data_msg;
                int[] shape = rov_localization.getAShape();
                if(shape[0]*shape[1] == param_data.size()){
                    SimpleMatrix data_A = new SimpleMatrix(shape[0],shape[1]);
                    for(int i = 0; i < shape[0]; i++) {
                        for (int j = 0; j < shape[1]; j++) {
                            data_A.set(i, j, (double) param_data.get(i * shape[1] + j));
                        }
                    }
                    rov_localization.setAData(data_A);
                }else{
                    Log.e("ROV_ERROR", "Localization: Matrix A size");
                }
            }
        });

        param_tree.addParameterListener("/dynamics_B", new ParameterListener() {
            @Override
            public void onNewValue(Object B_data_msg) {
                ArrayList<Number> param_data = (ArrayList<Number>) B_data_msg;
                int[] shape = rov_localization.getBShape();
                if(shape[0]*shape[1] == param_data.size()){
                    SimpleMatrix data_B = new SimpleMatrix(shape[0],shape[1]);
                    for(int i = 0; i < shape[0]; i++) {
                        for (int j = 0; j < shape[1]; j++) {
                            data_B.set(i, j, (double) param_data.get(i * shape[1] + j));
                        }
                    }
                    rov_localization.setBData(data_B);
                }else{
                    Log.e("ROV_ERROR", "Localization: Matrix B size");
                }
            }
        });

        param_tree.addParameterListener("/map", new ParameterListener() {
            @Override
            public void onNewValue(Object param_data) {}
        });
    }
}