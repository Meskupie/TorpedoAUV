package Communication;

import android.util.Log;

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
import org.ros.rosjava_geometry.Quaternion;

import Autonomy.Localization.CameraTarget;
import sensor_msgs.PointCloud;
import std_msgs.Float64;
import std_msgs.Float64MultiArray;
import std_msgs.Int32;
import std_msgs.Int32MultiArray;

public class CommunicationNode extends AbstractNodeMain {

    private int status_system;
    private int status_communication;

//    private boolean switch_front = false;
//    private boolean switch_center = false;
//    private boolean switch_rear = false;

    public double ui_battery_current = 0.0;
    public double ui_battery_voltage = 0.0;
    public int ui_battery_soc = 0;

    private Time time_current;
    private Time time_input_thrust;
    private Time time_status_system;
    private Time time_embedded;
    private Duration timeout_input_thrust = new Duration(0.15);
    private Duration timeout_status_system= new Duration(0.15);
    private Duration timeout_embedded = new Duration(0.15);

    private boolean ready_new_embedded = false;

    //Publishers
    private Publisher<Int32> status_communication_pub;
    private Publisher<Int32> status_embedded_pub;
    private Publisher<Int32> status_cameras_pub;
    private Subscriber<Int32> status_system_sub;
    private Subscriber<Float64MultiArray> input_thrust_sub;
    private ParameterTree param_tree;

    private Publisher<geometry_msgs.Quaternion> embedded_imu_pub;
    private Publisher<Float64> embedded_depth_pub;
    private Publisher<Float64> embedded_temperature_pub;
    private Publisher<Float64MultiArray> embedded_thrust_pub;
    private Publisher<Int32MultiArray> embedded_controller_states_pub;
    private Publisher<Int32> embedded_battery_soc_pub;
    private Publisher<Int32> embedded_battery_sop_pub;
    private Publisher<Float64> embedded_battery_voltage_pub;
    private Publisher<Float64> embedded_battery_current_pub;
    private Publisher<Int32> embedded_reed_switches_pub;
    private Publisher<PointCloud> camera_targets_front_pub;
    private Publisher<PointCloud> camera_targets_rear_pub;

    // Define publishing messages
    private Int32 status_communication_msg;
    private Int32 status_embedded_msg;
    private Int32 status_cameras_msg;
    private  geometry_msgs.Quaternion embedded_imu_msg;
    private Float64 embedded_temperature_msg;
    private Float64 embedded_depth_msg;
    private Float64MultiArray embedded_thrust_msg;
    private Int32MultiArray embedded_controller_states_msg;
    private Int32 embedded_battery_soc_msg;
    private Int32 embedded_battery_sop_msg;
    private Float64 embedded_battery_voltage_msg;
    private Float64 embedded_battery_current_msg;
    private Int32 embedded_reed_switches_msg;
    private PointCloud camera_targets_front_msg;
    private PointCloud camera_targets_rear_msg;

    private boolean ready_pub = false;

    // USB communication
    private boolean ready_usb_smc = false;
    private boolean ready_usb_front_cam = false;
    private boolean ready_usb_rear_cam = false;
    private USBDeviceWrapper usb_smc = new USBDeviceWrapper();
    private USBDeviceWrapper usb_front_cam = new USBDeviceWrapper();
    private USBDeviceWrapper usb_rear_cam = new USBDeviceWrapper();

    private MessageManager message_manager = new MessageManager();

    private String[] smc_status_enum = new String[]{"RUNNING","IDLE","STARTUP","TIMEOUT","FAULT", "FAULT_IMU","FAULT_BATTERY","FAULT MOTORS"};
    private String[] smc_motor_status_enum = new String[]{"COMM_FAILURE","IDLE,STARTUP","VALIDATION","STOP","START,RUN","ALIGNMENT","SPEEDFBKERROR","OVERCURRENT","STARTUP_FAILURE","STARTUP_BEMF_FAILURE","LF_TIMER_FAILURE","WD_RESET"};

    public CommunicationNode(){
        status_system = 0;
    }

    @Override
    public GraphName getDefaultNodeName() {return GraphName.of("CommunicationNode");}

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        // Define system connections
        status_communication_pub = connectedNode.newPublisher("status_communication", Int32._TYPE);
        status_embedded_pub = connectedNode.newPublisher("status_embedded", Int32._TYPE);
        status_cameras_pub = connectedNode.newPublisher("status_cameras", Int32._TYPE);
        status_system_sub = connectedNode.newSubscriber("status_system", Int32._TYPE);
        input_thrust_sub = connectedNode.newSubscriber("input_thrust",Float64MultiArray._TYPE);
        param_tree = connectedNode.getParameterTree();

        // Define data connections
        embedded_imu_pub = connectedNode.newPublisher("embedded_imu", geometry_msgs.Quaternion._TYPE);
        embedded_depth_pub = connectedNode.newPublisher("embedded_depth", Float64._TYPE);
        embedded_temperature_pub = connectedNode.newPublisher("embedded_temperature", Float64._TYPE);
        embedded_thrust_pub = connectedNode.newPublisher("embedded_thrust",Float64MultiArray._TYPE);
        embedded_controller_states_pub = connectedNode.newPublisher("embedded_controller_states",Int32MultiArray._TYPE);
        embedded_battery_soc_pub = connectedNode.newPublisher("embedded_battery_soc",Int32._TYPE);
        embedded_battery_sop_pub = connectedNode.newPublisher("embedded_battery_sop",Int32._TYPE);
        embedded_battery_voltage_pub = connectedNode.newPublisher("embedded_battery_voltage",Float64._TYPE);
        embedded_battery_current_pub = connectedNode.newPublisher("embedded_battery_current",Float64._TYPE);
        embedded_reed_switches_pub = connectedNode.newPublisher("embedded_reed_switches",Int32._TYPE);
        camera_targets_front_pub = connectedNode.newPublisher("camera_targets_front", PointCloud._TYPE);
        camera_targets_rear_pub = connectedNode.newPublisher("camera_targets_rear", PointCloud._TYPE);


        connectedNode.executeCancellableLoop(new CancellableLoop() {

            @Override protected void setup(){
                time_input_thrust = connectedNode.getCurrentTime();
                time_status_system = connectedNode.getCurrentTime();
                time_embedded = connectedNode.getCurrentTime();

                status_communication_msg = status_communication_pub.newMessage();
                status_embedded_msg = status_embedded_pub.newMessage();
                status_cameras_msg = status_cameras_pub.newMessage();
                embedded_imu_msg = embedded_imu_pub.newMessage();
                embedded_depth_msg = embedded_depth_pub.newMessage();
                embedded_temperature_msg = embedded_temperature_pub.newMessage();
                embedded_thrust_msg = embedded_thrust_pub.newMessage();
                embedded_controller_states_msg = embedded_controller_states_pub.newMessage();
                embedded_battery_soc_msg = embedded_battery_soc_pub.newMessage();
                embedded_battery_sop_msg = embedded_battery_sop_pub.newMessage();
                embedded_battery_voltage_msg = embedded_battery_voltage_pub.newMessage();
                embedded_battery_current_msg = embedded_battery_current_pub.newMessage();
                embedded_reed_switches_msg = embedded_reed_switches_pub.newMessage();
                camera_targets_front_msg = camera_targets_front_pub.newMessage();
                camera_targets_rear_msg = camera_targets_rear_pub.newMessage();

                ready_pub = true;
            }

            @Override
            protected void loop() throws InterruptedException {
                // Update USB devices
                if((usb_smc.usbDevice != null)&&(usb_smc.serial != null)){
                    ready_usb_smc = true;
                }
                if((usb_front_cam.usbDevice != null)&&(usb_front_cam.serial != null)){
                    ready_usb_front_cam = true;
                }
                if((usb_front_cam.usbDevice != null)&&(usb_front_cam.serial != null)){
                    ready_usb_rear_cam = true;
                }

                // Update embedded timeout
                if(ready_new_embedded){
                    time_embedded = connectedNode.getCurrentTime();
                    ready_new_embedded = false;
                }

                time_current = connectedNode.getCurrentTime();
                // Check timeouts
                if(time_current.compareTo(time_status_system.add(timeout_status_system)) == 1){
                    if(status_system > 0){Log.e("ROV_ERROR", "Communication node: Timeout on system state");}
                    status_communication |= 2;
                } else { status_communication &= ~2;}
//                if(time_current.compareTo(time_input_thrust.add(timeout_input_thrust)) == 1){
//                    if(status_system > 0){Log.e("ROV_ERROR", "Communication node: Timeout on input thrust");}
//                    status_communication |= 2;
//                } else { status_communication &= ~2;}
                if(time_current.compareTo(time_embedded.add(timeout_embedded)) == 1){
                    if(status_system > 0){Log.e("ROV_ERROR", "Embedded: Timeout on sensors message");}
                    status_communication |= 2;
                } else { status_communication &= ~2;}


                status_communication_msg.setData(status_communication);
                status_communication_pub.publish(status_communication_msg);
                Thread.sleep(10);
            }
        });

        status_system_sub.addMessageListener(new MessageListener<Int32>() {
            @Override public void onNewMessage(Int32 status_system_msg) {
                time_status_system = connectedNode.getCurrentTime();
                status_system = status_system_msg.getData();
                // request data from systems
                if(ready_usb_smc) {
                    new SerialWrite().execute(new Object[]{usb_smc.serial, message_manager.msg_smc_sensors.getRequest()});
                }
                if(ready_usb_front_cam) {
                    Log.d("ROV_LOG","Bad serial, bad");
                    new SerialWrite().execute(new Object[]{usb_front_cam.serial, message_manager.msg_front_targets.getRequest()});
                }
                if(ready_usb_rear_cam) {
                    Log.d("ROV_LOG","Bad serial, bad");
                    new SerialWrite().execute(new Object[]{usb_rear_cam.serial, message_manager.msg_rear_targets.getRequest()});
                }
            }
        });

        input_thrust_sub.addMessageListener(new MessageListener<Float64MultiArray>() {
            @Override
            public void onNewMessage(Float64MultiArray input_thrust_msg) {
                time_input_thrust = connectedNode.getCurrentTime();
                if (status_system >= 3) {
                    double[] thrusts = input_thrust_msg.getData();
                    message_manager.msg_smc_motors.input_thrust = thrusts;
                    new SerialWrite().execute(new Object[]{usb_smc.serial, message_manager.msg_smc_motors.getBuiltMsg()});
                }
            }
        });
    }

    public void SMCSensorsPub(MessageManager.MsgSMCSensors message) {
        ready_new_embedded = true;
        if(!ready_pub) {
            return;
        }
        status_embedded_msg.setData(message.smc_status);
        embedded_imu_msg.setX(message.imu.getX());
        embedded_imu_msg.setY(message.imu.getY());
        embedded_imu_msg.setZ(message.imu.getZ());
        embedded_imu_msg.setW(message.imu.getW());
        embedded_depth_msg.setData(0);//TODO: CHANGE THIS BACK TO: message.depth);
        embedded_temperature_msg.setData(message.temperature);
        embedded_thrust_msg.setData(message.motor_thrust);
        embedded_controller_states_msg.setData(message.motor_status);
        embedded_battery_soc_msg.setData(message.battery_SOC);
        embedded_battery_sop_msg.setData(message.battery_SOP);
        embedded_battery_voltage_msg.setData(message.battery_voltage);
        embedded_battery_current_msg.setData(message.battery_current);
        embedded_reed_switches_msg.setData(((message.switch_front?1:0)<<2)|((message.switch_center?1:0)<<1)|(message.switch_rear?1:0));

        // Publish the data
        status_embedded_pub.publish(status_embedded_msg);
        embedded_imu_pub.publish(embedded_imu_msg);
        embedded_depth_pub.publish(embedded_depth_msg);
        embedded_temperature_pub.publish(embedded_temperature_msg);
        embedded_thrust_pub.publish(embedded_thrust_msg);
        embedded_controller_states_pub.publish(embedded_controller_states_msg);
        embedded_battery_soc_pub.publish(embedded_battery_soc_msg);
        embedded_battery_sop_pub.publish(embedded_battery_sop_msg);
        embedded_battery_voltage_pub.publish(embedded_battery_voltage_msg);
        embedded_battery_current_pub.publish(embedded_battery_current_msg);
        embedded_reed_switches_pub.publish(embedded_reed_switches_msg);

        //set up reed switch values
//        switch_front = message.switch_front;
//        switch_center = message.switch_center;
//        switch_rear = message.switch_rear;

        ui_battery_current = message.battery_current;
        ui_battery_voltage = message.battery_voltage;
        ui_battery_soc = message.battery_SOC;

        //Log.d("ROV_LOG", "SMC Status: "+smc_status_enum[message.smc_status]+" Reed Switches: "+embedded_reed_switches_msg.getData());
    }

    public void frontCameraPub(MessageManager.MsgCameraTargets message){
//        camera_targets_front_msg.setChannels(); message.azumuths
//        for(int i = 0; i < target_cloud.getChannels().get(0).getValues().length; i++){
//            CameraTarget[] camera_targets = new CameraTarget[target_cloud.getChannels().get(0).getValues().length];
//            float[] azumuths  = target_cloud.getChannels().get(0).getValues();
//            float[] altitudes = target_cloud.getChannels().get(1).getValues();
//            float[] ids       = target_cloud.getChannels().get(2).getValues();
//            camera_targets[i] = new CameraTarget(azumuths[i],altitudes[i],(int)ids[i]);
//            rov_localization.setCameraTargetsRear(camera_targets);
//            attemptLocalizationUpdate(connectedNode);
//        }
    }

    public void rearCameraPub(MessageManager.MsgCameraTargets message){

    }

//    public boolean getReedFront() {
//        return this.switch_front;
//    }
//
//    public boolean getReedCenter() {
//        return this.switch_center;
//    }
//
//    public boolean getReedRear() {
//        return this.switch_rear;
//    }

    // Mutators
    public void setUSBSMC(USBDeviceWrapper device) {
        usb_smc = device;
    }
    public void setUSBFrontCam(USBDeviceWrapper device) {
        usb_front_cam = device;
    }
    public void setUSBRearCam(USBDeviceWrapper device) {
        usb_rear_cam = device;
    }

}
