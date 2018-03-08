package com.github.rosjava.android_apps.teleop;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;


/**
 * Created by meskupie on 07/03/18.
 */

// system_state topic enumeration:
// 0: Error
// 1: Idle
// 2: Initialize System
// 3: Arming
// 4: Running
// 5:
// 6:
// 7:

public class SystemNode extends AbstractNodeMain {
    private int system_state;

    private Time current_time;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("SystemNode");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        // Define ROS connections
        final Publisher<std_msgs.Int32> system_state_pub = connectedNode.newPublisher("system_state",std_msgs.Int32._TYPE);

        // Main cancelable loop
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            std_msgs.Int32 system_state_msg = system_state_pub.newMessage();

            //@Override protected void setup() {}
            @Override
            protected void loop() throws InterruptedException {
                //current_time = connectedNode.getCurrentTime();

                // Verify status and timeouts of all nodes

                // Publish system state
                system_state_msg.setData(system_state);
                system_state_pub.publish(system_state_msg);
                Thread.sleep(20);
            }
        });
    }
}
