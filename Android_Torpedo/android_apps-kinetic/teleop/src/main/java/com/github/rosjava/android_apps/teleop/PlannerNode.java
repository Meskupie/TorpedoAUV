package com.github.rosjava.android_apps.teleop;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;

/**
 * Created by meskupie on 07/03/18.
 */

public class PlannerNode extends AbstractNodeMain {

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("PlannerNode");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {


    }
}
