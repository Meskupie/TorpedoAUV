package com.github.rosjava.android_apps.teleop;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import geometry_msgs.Quaternion;
import sensor_msgs.PointCloud;
import std_msgs.Float64;
import std_msgs.Float64MultiArray;
import std_msgs.Int32;
import std_msgs.Int32MultiArray;

/**
 * Created by meskupie on 08/03/18.
 */

public class LoggingNode extends AbstractNodeMain {
//    // Define system connections
//
//    // System messages
//    private Subscriber<Int32> status_system_sub;
//    private Subscriber<std_msgs.Int32> status_communication_sub = connectedNode.newSubscriber("status_communication", Int32._TYPE);
//    private Subscriber<std_msgs.Int32> status_embedded_sub = connectedNode.newSubscriber("status_embedded", Int32._TYPE);
//    private Subscriber<std_msgs.Int32> status_camera_front_sub = connectedNode.newSubscriber("status_cameras", Int32._TYPE);
//    private Subscriber<std_msgs.Int32> status_camera_rear_sub = connectedNode.newSubscriber("status_cameras", Int32._TYPE);
//    private Subscriber<std_msgs.Int32> status_localization_sub = connectedNode.newSubscriber("status_communication", Int32._TYPE);
//    private Subscriber<std_msgs.Int32> status_planner_sub = connectedNode.newSubscriber("status_communication", Int32._TYPE);
//    private Subscriber<std_msgs.Int32> status_controller_sub = connectedNode.newSubscriber("status_communication", Int32._TYPE);
//    private Subscriber<std_msgs.Int32> status_parameters_sub = connectedNode.newSubscriber("status_communication", Int32._TYPE);
//
//    // Data
//    private Subscriber<Quaternion> embedded_imu_pub;
//    private Subscriber<Float64> embedded_depth_pub;
//    private Subscriber<Float64> embedded_temperature_pub;
//    private Subscriber<Float64MultiArray> embedded_thrust_pub;
//    private Subscriber<Int32MultiArray> embedded_controller_states_pub;
//    private Subscriber<Int32> embedded_battery_soc_pub;
//    private Subscriber<Int32> embedded_battery_sop_pub;
//    private Subscriber<Float64> embedded_battery_voltage_pub;
//    private Subscriber<Float64> embedded_battery_current_pub;
//    private Subscriber<Int32> embedded_reed_switches_pub;
//    private Subscriber<PointCloud> camera_targets_front_pub;
//    private Subscriber<PointCloud> camera_targets_rear_pub;
//
//    // Processed
//    private Subscriber<Float64MultiArray> input_thrust_sub;
//
//    private ParameterTree param_tree;



    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("LoggingNode");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {

    }
}
