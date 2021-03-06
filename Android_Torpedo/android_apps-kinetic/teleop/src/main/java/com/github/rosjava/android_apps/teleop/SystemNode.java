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
    public int status_system;
    private int status_system_prev;
    private int status_system_desired = 1;
    private boolean state_transition_ready = false;

    private Time time_start;
    private Time time_current;
    private int status_timeouts = 1;
    private int status_fault_nodes = 0;

    private Time time_state_transition;
    private Duration timeout_initial_leeway = new Duration(1);
    private Duration start_delay = new Duration(8);

    private Time time_run_begin;
    private Duration timeout_run = new Duration(2);

    private Time time_status_communication;
    private Time time_status_front_camera;
    private Time time_status_rear_camera;
    private Time time_status_embedded;
    private Time time_status_localization;
    private Time time_status_planner;
    private Time time_status_controller;
    private Time time_status_parameters;
    private Duration timeout_status_communication = new Duration(0.25);
    private Duration timeout_status_cameras = new Duration(0.25);
    private Duration timeout_status_embedded = new Duration(0.25);
    private Duration timeout_status_localization = new Duration(0.25);
    private Duration timeout_status_planner = new Duration(0.25);
    private Duration timeout_status_controller = new Duration(0.25);
    private Duration timeout_status_parameters = new Duration(3);
    private int status_communication;
    private int status_embedded;
    private int status_front_camera;
    private int status_rear_camera;
    private int status_localization;
    private int status_planner;
    private int status_controller;
    private int status_parameters;

    // Used for variable change listener
    public int getCurrentState(){
        return status_system;
    }
    public boolean isStateTransitionReady(){return state_transition_ready;}

    public boolean setDesiredState(int _status_system_desired){
        status_system_desired = _status_system_desired;
        return true;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("SystemNode");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        // Define system connections
        final Publisher<std_msgs.Int32> status_system_pub = connectedNode.newPublisher("status_system",std_msgs.Int32._TYPE);
        final Subscriber<std_msgs.Int32> status_communication_sub = connectedNode.newSubscriber("status_communication", Int32._TYPE);
        final Subscriber<std_msgs.Int32> status_embedded_sub = connectedNode.newSubscriber("status_embedded", Int32._TYPE);
        final Subscriber<std_msgs.Int32> status_front_camera_sub = connectedNode.newSubscriber("status_cameras", Int32._TYPE);
        final Subscriber<std_msgs.Int32> status_rear_camera_sub = connectedNode.newSubscriber("status_cameras", Int32._TYPE);
        final Subscriber<std_msgs.Int32> status_localization_sub = connectedNode.newSubscriber("status_localization", Int32._TYPE);
        final Subscriber<std_msgs.Int32> status_planner_sub = connectedNode.newSubscriber("status_planner", Int32._TYPE);
        final Subscriber<std_msgs.Int32> status_controller_sub = connectedNode.newSubscriber("status_controller", Int32._TYPE);
        final Subscriber<std_msgs.Int32> status_parameters_sub = connectedNode.newSubscriber("status_parameters", Int32._TYPE);
        final Subscriber<std_msgs.Int32> status_desired_sub = connectedNode.newSubscriber("status_desired", Int32._TYPE);

        // Define data connections

        // Main cancelable loop
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            std_msgs.Int32 status_system_msg = status_system_pub.newMessage();

            @Override
            protected void setup() {
                time_start = connectedNode.getCurrentTime();
                time_state_transition = connectedNode.getCurrentTime();
                time_status_communication = connectedNode.getCurrentTime();
                time_status_embedded = connectedNode.getCurrentTime();
                time_status_front_camera = connectedNode.getCurrentTime();
                time_status_rear_camera = connectedNode.getCurrentTime();
                time_status_localization = connectedNode.getCurrentTime();
                time_status_planner = connectedNode.getCurrentTime();
                time_status_controller = connectedNode.getCurrentTime();
                time_status_parameters = connectedNode.getCurrentTime();
                status_system = 0;
                status_system_prev = -1;
                status_timeouts = 243; // TODO change to 255 when cameras are added
            }

            @Override
            protected void loop() throws InterruptedException {
                time_current = connectedNode.getCurrentTime();

                // Verify timeouts of all nodes
                if (time_current.compareTo(time_status_communication.add(timeout_status_communication)) == 1) {
                    if (status_timeouts == 0) {
                        Log.e("ROV_ERROR", "System node: Timeout node communication");
                    }
                    status_timeouts |= 1;
                } else {
                    status_timeouts &= ~1;
                }
                if (time_current.compareTo(time_status_embedded.add(timeout_status_embedded)) == 1) {
                    if (status_timeouts == 0) {
                        Log.e("ROV_ERROR", "System node: Timeout node embedded");
                    }
                    status_timeouts |= 2;
                } else {
                    status_timeouts &= ~2;
                }
                //                if(time_current.compareTo(time_status_cameras.add(timeout_status_cameras)) == 1){
                //                    if(status_timeouts == 0){Log.e("ROV_ERROR", "System node: Timeout node camera");}
                //                    status_timeouts |= 4;
                //                } else { status_timeouts &= ~4;} // TODO: second camera
                if (time_current.compareTo(time_status_localization.add(timeout_status_localization)) == 1) {
                    if (status_timeouts == 0) {
                        Log.e("ROV_ERROR", "System node: Timeout node localization");
                    }
                    status_timeouts |= 16;
                } else {
                    status_timeouts &= ~16;
                }
                if (time_current.compareTo(time_status_planner.add(timeout_status_planner)) == 1) {
                    if (status_timeouts == 0) {
                        Log.e("ROV_ERROR", "System node: Timeout node planner");
                    }
                    status_timeouts |= 32;
                } else {
                    status_timeouts &= ~32;
                }
                if (time_current.compareTo(time_status_controller.add(timeout_status_controller)) == 1) {
                    if (status_timeouts == 0) {
                        Log.e("ROV_ERROR", "System node: Timeout node controller");
                    }
                    status_timeouts |= 64;
                } else {
                    status_timeouts &= ~64;
                }
                if (time_current.compareTo(time_status_parameters.add(timeout_status_parameters)) == 1) {
                    if (status_timeouts == 0) {
                        Log.e("ROV_ERROR", "System node: Timeout node parameters");
                    }
                    status_timeouts |= 128;
                } else {
                    status_timeouts &= ~128;
                }

                // Verify status of all nodes
                status_fault_nodes = 0;
                status_fault_nodes |= ((status_communication & 126) > 0 ? 1 : 0);
                status_fault_nodes |= (((status_embedded >= 4)||((status_embedded > 0)&&(status_system > 3))) ? 1 : 0) << 1;
                status_fault_nodes |= ((status_front_camera & 126) > 0 ? 1 : 0) << 2;
                status_fault_nodes |= ((status_rear_camera & 126) > 0 ? 1 : 0) << 3;
                status_fault_nodes |= ((status_localization & 126) > 0 ? 1 : 0) << 4;
                status_fault_nodes |= ((status_planner & 126) > 0 ? 1 : 0) << 5;
                status_fault_nodes |= ((status_controller & 126) > 0 ? 1 : 0) << 6;
                status_fault_nodes |= ((status_parameters & 126) > 0 ? 1 : 0) << 7;

                // Wait for boot time to start state machine
                if(time_current.compareTo(time_start.add(start_delay)) == 1){
                    // State machine
                    time_current = connectedNode.getCurrentTime();
                    switch (status_system) {
                        case -1: // Error
                            // To idle
                            if (((status_fault_nodes == 0) && (status_timeouts == 0))) {
                                status_system = 1;
                            }
                            break;
                        case 0: // Startup
                            // On entry
                            if (status_system != status_system_prev) {
                                time_state_transition = connectedNode.getCurrentTime();
                            }
                            // Transitions
                            // To idle
                            if ((status_fault_nodes == 0) && (status_timeouts == 0)) {
                                status_system = 1;
                            }
                            // To error (if an error occurs after leeway or error requested)
                            if (((time_current.compareTo(time_state_transition.add(timeout_initial_leeway)) == 1)
                                    && ((status_fault_nodes > 0) || (status_timeouts > 0)))
                                    || (status_system_desired == -1)) {
                                status_system = -1; // TODO: make -1
                            }
                            break;
                        case 1: // Idle
                            // Transitions
                            // To pose lock
                            if (status_system_desired == 2) {
                                status_system = 2;
                            }
                            // To error
                            if (((status_fault_nodes > 0) || (status_timeouts > 0)) || (status_system_desired == -1)) {
                                status_system = -1; // TODO: make -1
                            }
                            break;
                        case 2: // Pose Lock
                            // To arm
                            if ((status_localization & 128) == 0){
                                state_transition_ready = true;
                                if(status_system_desired == 3) {
                                    status_system = 3;
                                }
                            }else{
                                state_transition_ready = false;
                            }
                            // Return to home
                            if(status_system_desired == 1){
                                status_system = 1;
                            }
                            // To error
                            if (((status_fault_nodes > 0) || (status_timeouts > 0)) || (status_system_desired == -1)) {
                                status_system = -1;
                            }
                            // On exit
                            if (status_system != 2) {
                                state_transition_ready = false;
                            }
                            break;
                        case 3: // Arm and
                            // to Position hold
                            if ((status_embedded  == 0) && ((status_planner & 128) == 0)){
                                state_transition_ready = true;
                                if((status_system_desired == 4)||(status_system_desired == 5)) {
                                    status_system = 4;

                                }
                            }else{
                                state_transition_ready = false;
                            }
                            // Return to home
                            if(status_system_desired == 1){
                                status_system = 1;
                            }
                            // To error
                            if (((status_fault_nodes > 0) || (status_timeouts > 0)) || (status_system_desired == -1)) {
                                status_system = -1;
                            }
                            // On exit
                            if (status_system != 3) {
                                state_transition_ready = false;
                            }
                            break;
                        case 4: // Position Hold
                            if (status_system_desired == 5) {
                                status_system = 5;
                                time_run_begin = connectedNode.getCurrentTime();
                            }
                            // Return to home
                            if(status_system_desired == 1){
                                status_system = 1;
                            }
                            // To error
                            if (((status_fault_nodes > 0) || (status_timeouts > 0)) || (status_system_desired == -1)) {
                                status_system = -1;
                            }
                            break;
                        case 5: // Run
                            if (status_system != status_system_prev){
                                time_run_begin = connectedNode.getCurrentTime();
                            }else {
                                if (time_current.compareTo(time_run_begin.add(timeout_run)) == 1) {
                                    status_system = 6;
                                }
                            }

                            // Return to home
                            if(status_system_desired == 1){
                                status_system = 1;
                            }
                            // To error
                            if (((status_fault_nodes > 0) || (status_timeouts > 0)) || (status_system_desired == -1)) {
                                status_system = -1;
                            }
                            break;
                        case 6: // tele
                            // Return to home
                            if(status_system_desired == 1){
                                status_system = 1;
                            }
                            // To error
                            if (((status_fault_nodes > 0) || (status_timeouts > 0)) || (status_system_desired == -1)) {
                                status_system = -1;
                            }
                            break;
                        default:
                            // On entry
                            if (status_system != status_system_prev) {
                            }

                            // Transitions
                            status_system = -1;

                            // On exit
                            if (status_system != 0) {
                            }
                            break;
                    }
                    status_system_prev = status_system;
                }

                Log.d("DEBUG_MSG", "State: "+status_system+" Desired: "+status_system_desired+" Fault nodes: " + status_fault_nodes + " Timeouts: " + status_timeouts);

                // Publish system state
                status_system_msg.setData(status_system);
                status_system_pub.publish(status_system_msg);
                Thread.sleep(50);
            }
        });

        // Node status callbacks
        status_communication_sub.addMessageListener(new MessageListener<Int32>() {
            @Override public void onNewMessage(Int32 status_communication_msg) {
                time_status_communication = connectedNode.getCurrentTime();
                status_communication = status_communication_msg.getData();}});

        status_embedded_sub.addMessageListener(new MessageListener<Int32>() {
            @Override public void onNewMessage(Int32 status_embedded_msg) {
                time_status_embedded = connectedNode.getCurrentTime();
                status_embedded = status_embedded_msg.getData();}});

        status_front_camera_sub.addMessageListener(new MessageListener<Int32>() {
            @Override public void onNewMessage(Int32 status_cameras_msg) {
                time_status_front_camera = connectedNode.getCurrentTime();
                status_front_camera = status_cameras_msg.getData();}});

        status_rear_camera_sub.addMessageListener(new MessageListener<Int32>() {
            @Override public void onNewMessage(Int32 status_cameras_msg) {
                time_status_rear_camera = connectedNode.getCurrentTime();
                status_rear_camera = status_cameras_msg.getData();}});

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

        status_desired_sub.addMessageListener(new MessageListener<Int32>() {
            @Override public void onNewMessage(Int32 status_desired_msg) {
                status_system_desired = status_desired_msg.getData();}});
    }
}

// status_system topic enumeration:
// -1: Error
//
// 0: Startup
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
// bit 3: Error
// bit 7: Not ready for next state
