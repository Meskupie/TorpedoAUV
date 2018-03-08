package com.github.rosjava.android_apps.teleop;

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

import std_msgs.Float64MultiArray;
import std_msgs.Int32;

public class LocalizationNode extends AbstractNodeMain{
    private Localization rov_localization;

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
        final ParameterTree param_tree = connectedNode.getParameterTree();

        connectedNode.executeCancellableLoop(new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {


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

//import org.ros.rosjava_geometry.Quaternion;
//    import org.ros.rosjava_geometry.Transform;
//    import org.ros.rosjava_geometry.Vector3;
//        Transform test_start = new Transform(new Vector3(0,0,0),new Quaternion(0,0,0.383,0.924));
//        Transform test_delta = new Transform(new Vector3(1,0,0),new Quaternion(0,0,0,1));
//        Transform test_finish = test_start.multiply(test_delta);
//
//        Log.d("DEBUG_MSG", "x:"+test_finish.getTranslation().getX()+", y:"+test_finish.getTranslation().getY()+", z:"+test_finish.getTranslation().getZ());
//        Log.d("DEBUG_MSG","x:"+test_finish.getRotationAndScale().getX()+", y:"+test_finish.getRotationAndScale().getY()+", z:"+test_finish.getRotationAndScale().getZ()+", w:"+test_finish.getRotationAndScale().getW());
//
