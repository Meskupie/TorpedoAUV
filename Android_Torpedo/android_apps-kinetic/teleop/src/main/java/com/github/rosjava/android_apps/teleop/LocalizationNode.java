package com.github.rosjava.android_apps.teleop;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

/**
 * Created by meskupie on 2018-03-06.
 */

public class LocalizationNode extends AbstractNodeMain{
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("LocalizationNode");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {

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
