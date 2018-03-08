package com.github.rosjava.android_apps.teleop;

import android.util.Log;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.Duration;
import org.ros.message.MessageListener;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import std_msgs.Int32;

public class SystemNode extends AbstractNodeMain {
    private int system_state;
    private int system_state_prev;

    private Time time_current;
    private int status_timeouts;

    private Time time_state_transition;
    private Duration timeout_state_transition_leeway= new Duration(0.04);

    private Time time_status_communication;
    private Time time_status_embedded;
    private Time time_status_localization;
    private Time time_status_planner;
    private Time time_status_controller;
    private Time time_status_parameters;
    private Duration timeout_status_communication = new Duration(0.04);
    private Duration timeout_status_embedded = new Duration(0.04);
    private Duration timeout_status_localization = new Duration(0.04);
    private Duration timeout_status_planner = new Duration(0.04);
    private Duration timeout_status_controller = new Duration(0.04);
    private Duration timeout_status_parameters = new Duration(2);
    private int status_communication;
    private int status_embedded;
    private int status_localization;
    private int status_planner;
    private int status_controller;
    private int status_parameters;


    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("SystemNode");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        // Define ROS connections
        final Publisher<std_msgs.Int32> system_state_pub = connectedNode.newPublisher("system_state",std_msgs.Int32._TYPE);
        //final Subscriber<>

        // Node verification
        final Subscriber<std_msgs.Int32> status_communication_sub = connectedNode.newSubscriber("status_communication", Int32._TYPE);
        final Subscriber<std_msgs.Int32> status_embedded_sub = connectedNode.newSubscriber("status_embedded", Int32._TYPE);
        final Subscriber<std_msgs.Int32> status_localization_sub = connectedNode.newSubscriber("status_communication", Int32._TYPE);
        final Subscriber<std_msgs.Int32> status_planner_sub = connectedNode.newSubscriber("status_communication", Int32._TYPE);
        final Subscriber<std_msgs.Int32> status_controller_sub = connectedNode.newSubscriber("status_communication", Int32._TYPE);
        final Subscriber<std_msgs.Int32> status_parameters_sub = connectedNode.newSubscriber("status_communication", Int32._TYPE);

        // Main cancelable loop
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            std_msgs.Int32 system_state_msg = system_state_pub.newMessage();

            @Override
            protected void setup() {
                time_state_transition = connectedNode.getCurrentTime();
                time_status_communication = connectedNode.getCurrentTime();
                time_status_embedded = connectedNode.getCurrentTime();
                time_status_localization = connectedNode.getCurrentTime();
                time_status_planner = connectedNode.getCurrentTime();
                time_status_controller = connectedNode.getCurrentTime();
                time_status_parameters = connectedNode.getCurrentTime();
                system_state = 0;
                system_state_prev = -1;
            }

            @Override
            protected void loop() throws InterruptedException {
                time_current = connectedNode.getCurrentTime();

                // Verify timeouts of all nodes
                if(time_current.compareTo(time_status_communication.add(timeout_status_communication)) == 1){
                    Log.e("ROV_ERROR", "System node: Communication node timeout");
                    status_timeouts |= 1;
                } else { status_timeouts &= ~1;}
                if(time_current.compareTo(time_status_embedded.add(timeout_status_embedded)) == 1){
                    Log.e("ROV_ERROR", "System node: Embedded node timeout");
                    status_timeouts |= 2;
                } else { status_timeouts &= ~2;}
                if(time_current.compareTo(time_status_localization.add(timeout_status_localization)) == 1){
                    Log.e("ROV_ERROR", "System node: Localization node timeout");
                    status_timeouts |= 4;
                } else { status_timeouts &= ~4;}
                if(time_current.compareTo(time_status_planner.add(timeout_status_planner)) == 1){
                    Log.e("ROV_ERROR", "System node: Planner node timeout");
                    status_timeouts |= 8;
                } else { status_timeouts &= ~8;}
                if(time_current.compareTo(time_status_controller.add(timeout_status_controller)) == 1){
                    Log.e("ROV_ERROR", "System node: Controller node timeout");
                    status_timeouts |= 16;
                } else { status_timeouts &= ~16;}
                if(time_current.compareTo(time_status_parameters.add(timeout_status_parameters)) == 1){
                    Log.e("ROV_ERROR", "System node: Parameters node timeout");
                    status_timeouts |= 32;
                } else { status_timeouts &= ~32;}

                // Verify status of all nodes



                // State machine
                switch (system_state){
                    case 0:
                        // On entry
                        if(system_state != system_state_prev){

                        }

                        // On exit
                        if(system_state != 0){

                        }
                        break;
                    case 1:

                        break;
                    case 2:

                        break;
                    case 3:

                        break;
                    case 4:

                        break;
                    case 5:

                        break;
                    default:

                        break;
                }


                // Publish system state
                system_state_msg.setData(system_state);
                system_state_pub.publish(system_state_msg);
                Thread.sleep(20);
            }
        });

        // Node status callbacks
        status_communication_sub.addMessageListener(new MessageListener<Int32>() {
            @Override public void onNewMessage(Int32 status_communication_msg) {
                time_status_communication = connectedNode.getCurrentTime();
                status_communication = status_communication_msg.getData();}});

        status_localization_sub.addMessageListener(new MessageListener<Int32>() {
            @Override public void onNewMessage(Int32 status_localization_msg) {
                time_status_localization = connectedNode.getCurrentTime();
                status_localization = status_localization_msg.getData();}});

        status_planner_sub.addMessageListener(new MessageListener<Int32>() {
            @Override public void onNewMessage(Int32 status_planner_msg) {
                time_status_planner = connectedNode.getCurrentTime();
                status_planner = status_planner_msg.getData();}});

        status_controller_sub.addMessageListener(new MessageListener<Int32>() {
            @Override public void onNewMessage(Int32 status_controller_msg) {
                time_status_controller = connectedNode.getCurrentTime();
                status_controller = status_controller_msg.getData();}});

        status_parameters_sub.addMessageListener(new MessageListener<Int32>() {
            @Override public void onNewMessage(Int32 status_parameters_msg) {
                time_status_parameters = connectedNode.getCurrentTime();
                status_parameters = status_parameters_msg.getData();}});
    }
}

// system_state topic enumeration:
// 0: Error/Startup (any node or embedded system down/reporting problems)
//      All nodes and embedded frozen but passing messages if possible
//      Communication: Publishing embedded messages. Sending requests to embedded
//      Localization:  Idle
//      Planner:       Idle
//      Controller:    Idle
//      Parameters:    Sending all params
// 1: Idle
//      All nodes alive and responding appropriately
//      Communication: Publishing embedded messages. Sending requests to embedded
//      Localization:  Idle
//      Planner:       Idle
//      Controller:    Idle
//      Parameters:    Sending all params
// 2: Initialize System
//      Communication: Publishing embedded messages. Sending requests to embedded
//      Localization:  Building vehicle pose and giving pose lock boolean
//      Planner:       Idle
//      Controller:    Idle
//      Parameters:    Locked param update
// 3: Arming (motor commands sending, all 0. Confirmation of embedded system)
//      Communication: Streaming all messages
//      Localization:  Running
//      Planner:       Running, feeding 0 reference
//      Controller:    Running, feeding 0 motor command
//      Parameters:    Locked param update
// 4: Position hold (holding state at start of path)
//      Communication: Streaming all messages
//      Localization:  Running
//      Planner:       Running, feeding reference to hold start position
//      Controller:    Running, limit on thrust
//      Parameters:    Locked param update
// 5: Running (running down path)
//      Communication: Streaming all messages
//      Localization:  Running
//      Planner:       Running
//      Controller:    Running
//      Parameters:    Locked param update



// node status enumeration
// bit 0: In Idle (based on system state)
// bit 1: Timeout in node
// bit 2: Missing data
// bit 3: Not ready for next state
