package com.github.rosjava.android_apps.teleop;

import android.util.Log;

import org.jboss.netty.handler.codec.marshalling.MarshallingEncoder;
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
import org.ros.rosjava_geometry.Transform;

import std_msgs.Float64MultiArray;
import std_msgs.Int32;
import visualization_msgs.Marker;

public class PlannerNode extends AbstractNodeMain {
    private Planner rov_planner = new Planner();

    private int system_state;
    private int status_planner;

    private Time time_current;
    private Time time_system_state;
    private Time time_state_pose;
    private Duration timeout_system_state = new Duration(0.04);
    private Duration timeout_state_pose = new Duration(0.04);

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("PlannerNode");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        // Define system connections
        final Publisher<Int32> status_planner_pub = connectedNode.newPublisher("status_controller", Int32._TYPE);
        final Subscriber<Int32> system_state_sub = connectedNode.newSubscriber("system_state", Int32._TYPE);
        final ParameterTree param_tree = connectedNode.getParameterTree();
        // Define data connections
        final Publisher<Float64MultiArray> state_reference_pub = connectedNode.newPublisher("state_reference", Float64MultiArray._TYPE);
        final Subscriber<geometry_msgs.Transform> state_pose_pub = connectedNode.newSubscriber("state_pose", geometry_msgs.Transform._TYPE);
        // Define visualization markers
        final Publisher<Marker> marker_path_position_pub = connectedNode.newPublisher("marker_path_position", Marker._TYPE);
        final Publisher<Marker> marker_path_attitude_pub = connectedNode.newPublisher("marker_path_attitude", Marker._TYPE);
        final Publisher<Marker> marker_pose_current_pub = connectedNode.newPublisher("marker_pose_current", Marker._TYPE);
        final Publisher<Marker> marker_pose_desired_pub = connectedNode.newPublisher("marker_pose_deisred", Marker._TYPE);
        // Main loop
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            Int32 status_planner_msg = status_planner_pub.newMessage();

            @Override protected void setup() {
                time_system_state = connectedNode.getCurrentTime();
                time_state_pose = connectedNode.getCurrentTime();
                system_state = 0;
            }

            @Override
            protected void loop() throws InterruptedException {
                // Set variables
                time_current = connectedNode.getCurrentTime();
                // Check timeouts
                if(time_current.compareTo(time_system_state.add(timeout_system_state)) == 1){
                    Log.e("ROV_ERROR", "Planner node: Timeout on system state");
                    status_planner |= 2;
                } else { status_planner &= ~2;}
                if(time_current.compareTo(time_state_pose.add(timeout_state_pose)) == 1){
                    Log.e("ROV_ERROR", "Planner node: Timeout on state pose");
                    status_planner |= 2;
                } else {status_planner &= ~2;}

                // Check compliance
                if(system_state <= 2){
                    status_planner |= 1;
                } else {status_planner &= ~1;}

                // Publish status
                status_planner_msg.setData(status_planner);
                status_planner_pub.publish(status_planner_msg);
                Thread.sleep(20);
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
    }
}
