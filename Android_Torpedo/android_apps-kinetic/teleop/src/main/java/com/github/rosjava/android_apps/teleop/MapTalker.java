package com.github.rosjava.android_apps.teleop;


import android.content.Context;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

/**
 * Created by isaiah on 07/02/18.
 */

public class MapTalker implements NodeMain {

    private MapGenerator mapGenerator;

    public MapTalker(Context context) {
        this.mapGenerator = new MapGenerator(context);
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("MapTalker/aTalker");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        final Publisher<std_msgs.String> publisher = connectedNode.newPublisher("aMapValues", "std_msgs/String");
        final String str = this.mapGenerator.toString();
        final std_msgs.String string = publisher.newMessage();
        string.setData(str);

        final CancellableLoop aLoop = new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {

                publisher.publish(string);
                Thread.sleep(100);
            }
        };

        connectedNode.executeCancellableLoop(aLoop);

    }

    @Override
    public void onShutdown(Node node) {

    }

    @Override
    public void onShutdownComplete(Node node) {

    }

    @Override
    public void onError(Node node, Throwable throwable) {

    }
}

