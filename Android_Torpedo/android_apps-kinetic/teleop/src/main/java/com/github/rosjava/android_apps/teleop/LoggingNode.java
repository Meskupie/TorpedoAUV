package com.github.rosjava.android_apps.teleop;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

/**
 * Created by meskupie on 08/03/18.
 */

public class LoggingNode extends AbstractNodeMain {
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("LoggingNode");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {

    }
}
