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
import org.ros.rosjava_geometry.Quaternion;

import Util.EmbeddedManager;
import sensor_msgs.PointCloud;
import std_msgs.Float32;
import std_msgs.Float64;
import std_msgs.Float64MultiArray;
import std_msgs.Int32;
import std_msgs.Int32MultiArray;

public class CommunicationNode extends AbstractNodeMain {

    private int status_system;
    private int status_communication;

    private Time time_current;
    private Time time_input_thrust;
    private Time time_status_system;
    private Duration timeout_input_thrust = new Duration(0.1);
    private Duration timeout_status_system= new Duration(0.1);


    // Accessible data
    private int status_embedded;
    private Quaternion embedded_imu;
    private double embedded_temperature;
    private double[] embedded_thrust;
    private int[] embedded_controller_states;
    private int embedded_battery_soc;
    private double embedded_battery_voltage;
    private int embedded_reed_switches;
    private double[] input_thrust;


    public CommunicationNode(){
        status_system = 0;
    }

    @Override
    public GraphName getDefaultNodeName() {return GraphName.of("CommunicationNode");}

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        // Define system connections
        final Publisher<Int32> status_communication_pub = connectedNode.newPublisher("status_communication", Int32._TYPE);
        final Publisher<Int32> status_embedded_pub = connectedNode.newPublisher("status_embedded", Int32._TYPE);
        final Publisher<Int32> status_cameras_pub = connectedNode.newPublisher("status_cameras", Int32._TYPE);
        final Subscriber<Int32> status_system_sub = connectedNode.newSubscriber("status_system", Int32._TYPE);
        final Subscriber<Float64MultiArray> input_thrust_sub = connectedNode.newSubscriber("input_thrust",Float64MultiArray._TYPE);
        final ParameterTree param_tree = connectedNode.getParameterTree();

        // Define data connections
        final Publisher<geometry_msgs.Quaternion> embedded_imu_pub = connectedNode.newPublisher("embedded_imu", geometry_msgs.Quaternion._TYPE);
        final Publisher<Float64> embedded_temperature_pub = connectedNode.newPublisher("embedded_temperature", Float64._TYPE);
        final Publisher<Float64MultiArray> embedded_thrust_pub = connectedNode.newPublisher("embedded_thrust",Float64MultiArray._TYPE);
        final Publisher<Int32MultiArray> embedded_controller_states_pub = connectedNode.newPublisher("embedded_controller_states",Int32MultiArray._TYPE);
        final Publisher<Int32> embedded_battery_soc_pub = connectedNode.newPublisher("embedded_battery_soc",Int32._TYPE);
        final Publisher<Float64> embedded_battery_voltage_pub = connectedNode.newPublisher("embedded_battery_voltage",Float64._TYPE);
        final Publisher<Int32> embedded_reed_switches_pub = connectedNode.newPublisher("embedded_reed_switchs",Int32._TYPE);
        final Publisher<PointCloud> camera_targets_pub = connectedNode.newPublisher("camera_targets", PointCloud._TYPE);


        connectedNode.executeCancellableLoop(new CancellableLoop() {
            // Define publishing messages
            Int32 status_communication_msg = status_communication_pub.newMessage();
            Int32 status_embedded_msg = status_embedded_pub.newMessage();
            Int32 status_cameras_msg = status_cameras_pub.newMessage();
            geometry_msgs.Quaternion embedded_imu_msg = embedded_imu_pub.newMessage();
            Float64 embedded_temperature_msg = embedded_temperature_pub.newMessage();
            Float64MultiArray embedded_thrust_msg = embedded_thrust_pub.newMessage();
            Int32MultiArray embedded_controller_states_msg = embedded_controller_states_pub.newMessage();
            Int32 embedded_battery_soc_msg = embedded_battery_soc_pub.newMessage();
            Float64 embedded_battery_voltage_msg = embedded_battery_voltage_pub.newMessage();
            Int32 embedded_reed_switches_msg = embedded_reed_switches_pub.newMessage();
            PointCloud camera_targets = camera_targets_pub.newMessage();

            @Override protected void setup(){
                time_input_thrust = connectedNode.getCurrentTime();
                time_status_system = connectedNode.getCurrentTime();
            }

            @Override
            protected void loop() throws InterruptedException {
                time_current = connectedNode.getCurrentTime();
                // Check timeouts
                if(time_current.compareTo(time_status_system.add(timeout_status_system)) == 1){
                    if(status_system >= 0){Log.e("ROV_ERROR", "Communication node: Timeout on system state");}
                    status_communication |= 2;
                } else { status_communication &= ~2;}
                if(time_current.compareTo(time_input_thrust.add(timeout_input_thrust)) == 1){
                    if(status_system >= 0){Log.e("ROV_ERROR", "Communication node: Timeout on input thrust");}
                    status_communication |= 2;
                } else { status_communication &= ~2;}


                status_communication_msg.setData(status_communication);
                status_communication_pub.publish(status_communication_msg);
                Thread.sleep(20);
            }
        });

        status_system_sub.addMessageListener(new MessageListener<Int32>() {
            @Override public void onNewMessage(Int32 status_system_msg) {
                time_status_system = connectedNode.getCurrentTime();
                status_system = status_system_msg.getData();

                // TODO: Add streaming of sensor request messages for Pis and Embedded
            }
        });

        input_thrust_sub.addMessageListener(new MessageListener<Float64MultiArray>() {
            @Override public void onNewMessage(Float64MultiArray input_thrust_msg) {
                time_input_thrust = connectedNode.getCurrentTime();
                input_thrust = input_thrust_msg.getData();

                if(status_system >= 3){
                    // TODO: Add streaming of thrust commands over USB
                }
            }
        });

    }
}
