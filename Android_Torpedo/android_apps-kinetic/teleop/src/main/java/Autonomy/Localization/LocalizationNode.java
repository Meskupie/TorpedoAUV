package Autonomy.Localization;

import android.graphics.Camera;
import android.util.Log;

import org.ejml.simple.SimpleMatrix;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.Duration;
import org.ros.message.MessageListener;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterListener;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;

import java.util.ArrayList;

import geometry_msgs.Twist;
import sensor_msgs.PointCloud;
import std_msgs.Float64;
import std_msgs.Float64MultiArray;
import std_msgs.Int32;

public class LocalizationNode extends AbstractNodeMain{
    private Localization rov_localization = new Localization();

    // Publishing variables
    private Publisher<geometry_msgs.Transform> state_pose_pub;
    private Publisher<Twist> state_twist_pub;
    private Publisher<Float64> state_fitness_pub;

    private geometry_msgs.Transform state_pose_msg;
    private geometry_msgs.Vector3 state_pose_msg_vect;
    private geometry_msgs.Quaternion state_pose_msg_quat;
    private Twist state_twist_msg;
    private geometry_msgs.Vector3 state_twist_msg_vect;
    private Float64 state_fitness_msg;

    private Transform state_pose;
    private Twist state_twist;
    private double state_fitness;

    private int status_system;
    private int status_localization;

    // State machine
    private Time time_current;
    private Time time_status_system;
    private Time time_embedded_thrust;
    private Time time_embedded_imu;
    private Time time_embedded_depth;
    private Time time_camera_targets_front;
    private Time time_camera_targets_rear;

    private Duration timeout_status_system = new Duration(0.04);
    private Duration timeout_embedded_thrust = new Duration(0.04);
    private Duration timeout_embedded_imu = new Duration(0.04);
    private Duration timeout_embedded_depth = new Duration(0.04);
    private Duration timeout_camera_targets_front = new Duration(0.04);
    private Duration timeout_camera_targets_rear = new Duration(0.04);

    private boolean attemptLocalizationUpdate(ConnectedNode connectedNode){
        if((status_localization&127) == 0){ // Nothing is preventing us from running
            if(((status_localization&128) == 128)||(status_system <= 2)){ // We do not have pose lock
                rov_localization.attemptInitialization(connectedNode.getCurrentTime());
            }else{ // We should be good to stream
                if(rov_localization.attemptUpdate(connectedNode.getCurrentTime())){
                    // Get and publish pose
                    state_pose = rov_localization.getPose();
                    state_pose_msg_vect.setX(state_pose.getTranslation().getX());
                    state_pose_msg_vect.setY(state_pose.getTranslation().getY());
                    state_pose_msg_vect.setZ(state_pose.getTranslation().getZ());
                    state_pose_msg.setTranslation(state_pose_msg_vect);
                    state_pose_msg_quat.setX(state_pose.getRotationAndScale().getX());
                    state_pose_msg_quat.setY(state_pose.getRotationAndScale().getY());
                    state_pose_msg_quat.setZ(state_pose.getRotationAndScale().getZ());
                    state_pose_msg_quat.setW(state_pose.getRotationAndScale().getW());
                    state_pose_msg.setRotation(state_pose_msg_quat);
                    state_pose_pub.publish(state_pose_msg);

                    // TODO: add this back in
//                    // Get and publish twist
//                    state_twist = rov_localization.getTwist();
//                    state_twist_msg_vect.setX(state_twist.getLinear().getX());
//                    state_twist_msg_vect.setY(state_twist.getLinear().getY());
//                    state_twist_msg_vect.setZ(state_twist.getLinear().getZ());
//                    state_twist_msg.setLinear(state_pose_msg_vect);
//                    state_twist_msg_vect.setX(state_twist.getAngular().getX());
//                    state_twist_msg_vect.setY(state_twist.getAngular().getY());
//                    state_twist_msg_vect.setZ(state_twist.getAngular().getZ());
//                    state_twist_msg.setAngular(state_pose_msg_vect);
//                    state_twist_pub.publish(state_twist_msg);
//
//                    // Get and publish fitness
//                    state_fitness = rov_localization.getFitness();
//                    state_fitness_msg.setData(state_fitness);
//                    state_fitness_pub.publish(state_fitness_msg);
                }
            }
        }
        else{ // We shouldn't be running
            rov_localization.resetInitialOrientation();
        }
        return true;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("LocalizationNode");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        // Define system connections
        final Publisher<Int32> status_localization_pub = connectedNode.newPublisher("status_localization",Int32._TYPE);
        final Subscriber<Int32> status_system_sub = connectedNode.newSubscriber("status_system", Int32._TYPE);
        final ParameterTree param_tree = connectedNode.getParameterTree();
        // Define data connections
        state_pose_pub = connectedNode.newPublisher("state_pose", geometry_msgs.Transform._TYPE);
        state_twist_pub = connectedNode.newPublisher("state_twist", Twist._TYPE);
        state_fitness_pub = connectedNode.newPublisher("state_fitness", Float64._TYPE);
        final Subscriber<Float64MultiArray> embedded_thrust_sub = connectedNode.newSubscriber("embedded_thrust", Float64MultiArray._TYPE);
        final Subscriber<geometry_msgs.Quaternion> embedded_imu_sub = connectedNode.newSubscriber("embedded_imu", geometry_msgs.Quaternion._TYPE);
        final Subscriber<Float64> embedded_depth_sub = connectedNode.newSubscriber("embedded_depth", Float64._TYPE);
        final Subscriber<PointCloud> camera_targets_front_sub = connectedNode.newSubscriber("camera_targets_front", PointCloud._TYPE);
        final Subscriber<PointCloud> camera_targets_rear_sub = connectedNode.newSubscriber("camera_targets_rear", PointCloud._TYPE);

        // Define message wrappers
        state_pose_msg = state_pose_pub.newMessage();
        state_pose_msg_vect = state_pose_pub.newMessage().getTranslation();
        state_pose_msg_quat = state_pose_pub.newMessage().getRotation();
        state_twist_msg = state_twist_pub.newMessage();
        state_twist_msg_vect = state_twist_pub.newMessage().getLinear();
        state_fitness_msg = state_fitness_pub.newMessage();


        connectedNode.executeCancellableLoop(new CancellableLoop() {
            Int32 status_localization_msg = status_localization_pub.newMessage();
            
            @Override
            protected void setup(){
                time_status_system = connectedNode.getCurrentTime();
                time_embedded_thrust = connectedNode.getCurrentTime();
                time_embedded_imu = connectedNode.getCurrentTime();
                time_embedded_depth = connectedNode.getCurrentTime();
                time_camera_targets_front = connectedNode.getCurrentTime();
                time_camera_targets_rear = connectedNode.getCurrentTime();
            }

            @Override
            protected void loop() throws InterruptedException {

                // Check compliance
                if(status_system <= 1){
                    status_localization |= 1;
                } else {status_localization &= ~1;}

                // Check timeouts
                time_current = connectedNode.getCurrentTime();
                if(time_current.compareTo(time_status_system.add(timeout_status_system)) == 1){
                    if(status_system > 0){Log.e("ROV_ERROR", "Localization node: Timeout on system state");}
                    status_localization |= 2;
                }else{status_localization &= ~2;}
                if(time_current.compareTo(time_embedded_thrust.add(timeout_embedded_thrust)) == 1){
                    if(status_system > 0){Log.e("ROV_ERROR", "Localization node: Timeout on embedded thrust");}
                    status_localization |= 2;
                }else{status_localization &= ~2;}
                if(time_current.compareTo(time_embedded_imu.add(timeout_embedded_imu)) == 1){
                    if(status_system > 0){Log.e("ROV_ERROR", "Localization node: Timeout on embedded imu");}
                    status_localization |= 2;
                }else{status_localization &= ~2;}
                if(time_current.compareTo(time_embedded_thrust.add(timeout_embedded_depth)) == 1){
                    if(status_system > 0){Log.e("ROV_ERROR", "Localization node: Timeout on embedded depth");}
                    status_localization |= 2;
                }else{ status_localization &= ~2;}
                //TODO: Uncomment when cameras are ready
                //if(time_current.compareTo(time_camera_targets_front.add(timeout_camera_targets_front)) == 1){
                //    if(status_system != 0){Log.e("ROV_ERROR", "Localization node: Timeout on camera_targets_front");}
                //    status_localization |= 2;
                //} else { status_localization &= ~2;}
                //if(time_current.compareTo(time_camera_targets_rear.add(timeout_camera_targets_rear)) == 1){
                //    if(status_system != 0){Log.e("ROV_ERROR", "Localization node: Timeout on camera_targets_rear");}
                //    status_localization |= 2;
                //} else { status_localization &= ~2;}

                // Check if all data/params filled
                if (!rov_localization.isReady()){
                    status_localization |= 4;
                }else{
                    status_localization &= ~4;
                }

                // Check errors
                // NOT IMPLEMENTED: This would modify status_localization bit 3

                // Check if the system is ready to proceed
                if(rov_localization.isLocked()){
                    status_localization &= ~128;
                }else{ status_localization |= 128;}

                attemptLocalizationUpdate(connectedNode);
                // Publish status
                status_localization_msg.setData(status_localization);
                status_localization_pub.publish(status_localization_msg);
                Thread.sleep(5);
            }
        });

        // System state callback
        status_system_sub.addMessageListener(new MessageListener<Int32>() {
            @Override
            public void onNewMessage(Int32 status_system_msg) {
                time_status_system = connectedNode.getCurrentTime();
                status_system = status_system_msg.getData();
                attemptLocalizationUpdate(connectedNode);
            }
        });

        embedded_imu_sub.addMessageListener(new MessageListener<geometry_msgs.Quaternion>() {
            @Override
            public void onNewMessage(geometry_msgs.Quaternion imu_data) {
                time_embedded_imu = connectedNode.getCurrentTime();
                rov_localization.setImuData(new Quaternion(imu_data.getX(),imu_data.getY(),imu_data.getZ(),imu_data.getW()));
                attemptLocalizationUpdate(connectedNode);
            }
        });

        embedded_thrust_sub.addMessageListener(new MessageListener<Float64MultiArray>() {
            @Override
            public void onNewMessage(Float64MultiArray embedded_thrust_msg) {
                time_embedded_thrust = connectedNode.getCurrentTime();
                SimpleMatrix embedded_thrust = new SimpleMatrix(6,1,false,embedded_thrust_msg.getData());
                rov_localization.setThrusterData(embedded_thrust);
                attemptLocalizationUpdate(connectedNode);
            }
        });

        camera_targets_front_sub.addMessageListener(new MessageListener<PointCloud>() {
            @Override
            public void onNewMessage(PointCloud target_cloud) {
                time_camera_targets_front = connectedNode.getCurrentTime();
                int size = target_cloud.getPoints().size();
                if(size == target_cloud.getChannels().size()){
                    for(int i = 0; i < size; i++){
                        CameraTarget[] camera_targets = new CameraTarget[size];
                        float[] azumuths  = target_cloud.getChannels().get(0).getValues();
                        float[] altitudes = target_cloud.getChannels().get(1).getValues();
                        float[] ids       = target_cloud.getChannels().get(2).getValues();
                        camera_targets[i] = new CameraTarget(azumuths[i],altitudes[i],(int)ids[i]);
                        rov_localization.setCameraTargetsFront(camera_targets);
                        attemptLocalizationUpdate(connectedNode);
                    }
                }else{
                    Log.e("ROV_ERROR", "Localization: camera targets size");
                }
            }
        });

        camera_targets_rear_sub.addMessageListener(new MessageListener<PointCloud>() {
            @Override
            public void onNewMessage(PointCloud target_cloud) {
                time_camera_targets_rear = connectedNode.getCurrentTime();
                int size = target_cloud.getPoints().size();
                if(size == target_cloud.getChannels().size()){
                    for(int i = 0; i < size; i++){
                        CameraTarget[] camera_targets = new CameraTarget[size];
                        float[] azumuths  = target_cloud.getChannels().get(0).getValues();
                        float[] altitudes = target_cloud.getChannels().get(1).getValues();
                        float[] ids       = target_cloud.getChannels().get(2).getValues();
                        camera_targets[i] = new CameraTarget(azumuths[i],altitudes[i],(int)ids[i]);
                        rov_localization.setCameraTargetsRear(camera_targets);
                        attemptLocalizationUpdate(connectedNode);
                    }
                }else{
                    Log.e("ROV_ERROR", "Localization: camera targets size");
                }
            }
        });

        param_tree.addParameterListener("/dynamics_A", new ParameterListener() {
            @Override
            public void onNewValue(Object A_data_msg) {
                ArrayList<Number> param_data = (ArrayList<Number>) A_data_msg;
                int[] shape = rov_localization.getAShape();
                if(shape[0]*shape[1] == param_data.size()){
                    SimpleMatrix data_A = new SimpleMatrix(shape[0],shape[1]);
                    for(int i = 0; i < shape[0]; i++) {
                        for (int j = 0; j < shape[1]; j++) {
                            data_A.set(i, j, (double) param_data.get(i * shape[1] + j));
                        }
                    }
                    rov_localization.setAData(data_A);
                }else{
                    Log.e("ROV_ERROR", "Localization: Matrix A size");
                }
            }
        });

        param_tree.addParameterListener("/dynamics_B", new ParameterListener() {
            @Override
            public void onNewValue(Object B_data_msg) {
                ArrayList<Number> param_data = (ArrayList<Number>) B_data_msg;
                int[] shape = rov_localization.getBShape();
                if(shape[0]*shape[1] == param_data.size()){
                    SimpleMatrix data_B = new SimpleMatrix(shape[0],shape[1]);
                    for(int i = 0; i < shape[0]; i++) {
                        for (int j = 0; j < shape[1]; j++) {
                            data_B.set(i, j, (double) param_data.get(i * shape[1] + j));
                        }
                    }
                    rov_localization.setBData(data_B);
                }else{
                    Log.e("ROV_ERROR", "Localization: Matrix B size");
                }
            }
        });

        param_tree.addParameterListener("/map", new ParameterListener() {
            @Override
            public void onNewValue(Object map_data_msg) {
                ArrayList<Number> map_data = (ArrayList<Number>) map_data_msg;
                int size = map_data.size()/4;
                if(size*4 == map_data.size()) {
                    MapTarget[] map = new MapTarget[size];
                    for (int i = 0; i < size; i++) {
                        map[i] = new MapTarget((double) map_data.get((i * 4) + 0), (double) map_data.get((i * 4) + 1), (double) map_data.get((i * 4) + 2), (int) map_data.get((i * 4) + 3));
                    }
                    rov_localization.setMapData(map);
                }else{
                    Log.e("ROV_ERROR", "Localization: Map size");
                }
            }
        });
    }
}