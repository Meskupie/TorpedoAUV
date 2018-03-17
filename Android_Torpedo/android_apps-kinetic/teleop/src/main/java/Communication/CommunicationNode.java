package Communication;

import android.util.Log;

import com.felhr.usbserial.UsbSerialDevice;

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

import Communication.AsyncArduinoWrite;
import Util.EmbeddedManager;
import Util.MotorOutputs;
import sensor_msgs.PointCloud;
import std_msgs.Float32;
import std_msgs.Float64;
import std_msgs.Float64MultiArray;
import std_msgs.Int32;
import std_msgs.Int32MultiArray;

public class CommunicationNode extends AbstractNodeMain {

    private int status_system;
    private int status_communication;

    private Time time_current;
    private Time time_input_thrust;
    private Time time_status_system;
    private Duration timeout_input_thrust = new Duration(0.1);
    private Duration timeout_status_system= new Duration(0.1);

    private static final int INT_MAX = (2<<15)-1;


    // Accessible data
    private int status_embedded;
    private Quaternion embedded_imu;
    private double embedded_temperature;
    private double[] embedded_thrust;
    private int[] embedded_controller_states;
    private int embedded_battery_soc;
    private double embedded_battery_voltage;
    private int embedded_reed_switches;
    private double[] input_thrust;

    //Publishers
    private Publisher<Int32> status_communication_pub;
    private Publisher<Int32> status_embedded_pub;
    private Publisher<Int32> status_cameras_pub;
    private Subscriber<Int32> status_system_sub;
    private Subscriber<Float64MultiArray> input_thrust_sub;
    private ParameterTree param_tree;

    // Define data connections Publishers
    private Publisher<geometry_msgs.Quaternion> embedded_imu_pub;
    private Publisher<Float64> embedded_temperature_pub;
    private Publisher<Float64MultiArray> embedded_thrust_pub;
    private Publisher<Int32MultiArray> embedded_controller_states_pub;
    private Publisher<Int32> embedded_battery_soc_pub;
    private Publisher<Float64> embedded_battery_voltage_pub;
    private Publisher<Int32> embedded_reed_switches_pub;
    private Publisher<PointCloud> camera_targets_pub;

    // Define publishing messages
    private Int32 status_communication_msg;
    private Int32 status_embedded_msg;
    private Int32 status_cameras_msg;
    private  geometry_msgs.Quaternion embedded_imu_msg;
    private Float64 embedded_temperature_msg;
    private Float64MultiArray embedded_thrust_msg;
    private Int32MultiArray embedded_controller_states_msg;
    private Int32 embedded_battery_soc_msg;
    private Float64 embedded_battery_voltage_msg;
    private Int32 embedded_reed_switches_msg;
    private PointCloud camera_targets;
    private boolean initialized = false;

    private int numberOfMessagesSent = 0;

    private UsbSerialDevice serial;


    public CommunicationNode(){
        status_system = 0;
    }

    public void setSerial(UsbSerialDevice serial) {
        this.serial = serial;
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
        embedded_temperature_pub = connectedNode.newPublisher("embedded_temperature", Float64._TYPE);
        embedded_thrust_pub = connectedNode.newPublisher("embedded_thrust",Float64MultiArray._TYPE);
        embedded_controller_states_pub = connectedNode.newPublisher("embedded_controller_states",Int32MultiArray._TYPE);
        embedded_battery_soc_pub = connectedNode.newPublisher("embedded_battery_soc",Int32._TYPE);
        embedded_battery_voltage_pub = connectedNode.newPublisher("embedded_battery_voltage",Float64._TYPE);
        embedded_reed_switches_pub = connectedNode.newPublisher("embedded_reed_switchs",Int32._TYPE);
        camera_targets_pub = connectedNode.newPublisher("camera_targets", PointCloud._TYPE);


        connectedNode.executeCancellableLoop(new CancellableLoop() {



            @Override protected void setup(){
                time_input_thrust = connectedNode.getCurrentTime();
                time_status_system = connectedNode.getCurrentTime();

                status_communication_msg = status_communication_pub.newMessage();
                status_embedded_msg = status_embedded_pub.newMessage();
                status_cameras_msg = status_cameras_pub.newMessage();
                embedded_imu_msg = embedded_imu_pub.newMessage();
                embedded_temperature_msg = embedded_temperature_pub.newMessage();
                embedded_thrust_msg = embedded_thrust_pub.newMessage();
                embedded_controller_states_msg = embedded_controller_states_pub.newMessage();
                embedded_battery_soc_msg = embedded_battery_soc_pub.newMessage();
                embedded_battery_voltage_msg = embedded_battery_voltage_pub.newMessage();
                embedded_reed_switches_msg = embedded_reed_switches_pub.newMessage();
                camera_targets = camera_targets_pub.newMessage();
                initialized = true;

            }

            @Override
            protected void loop() throws InterruptedException {
                time_current = connectedNode.getCurrentTime();
                // Check timeouts
                if(time_current.compareTo(time_status_system.add(timeout_status_system)) == 1){
                    if(status_system >= 0){Log.e("ROV_ERROR", "Communication node: Timeout on system state");}
                    status_communication |= 2;
                } else { status_communication &= ~2;}
                if(time_current.compareTo(time_input_thrust.add(timeout_input_thrust)) == 1){
                    if(status_system >= 0){Log.e("ROV_ERROR", "Communication node: Timeout on input thrust");}
                    status_communication |= 2;
                } else { status_communication &= ~2;}


                status_communication_msg.setData(status_communication);
                status_communication_pub.publish(status_communication_msg);
                Thread.sleep(20);
            }
        });

        status_system_sub.addMessageListener(new MessageListener<Int32>() {
            @Override public void onNewMessage(Int32 status_system_msg) {
                time_status_system = connectedNode.getCurrentTime();
                status_system = status_system_msg.getData();
                new AsyncArduinoWrite().execute(new Object[]{serial, EmbeddedManager.WRITE_COMMAND.getBytes()});

                int[] thrustVals = new int[]{23, -24, -95, 0, 70,100};
                byte[] bytes = EmbeddedManager.getEmbeddedByteArray(thrustVals, numberOfMessagesSent);
                numberOfMessagesSent++;
                new AsyncArduinoWrite().execute(new Object[]{serial, bytes});

            }
        });

        input_thrust_sub.addMessageListener(new MessageListener<Float64MultiArray>() {
            @Override
            public void onNewMessage(Float64MultiArray input_thrust_msg) {
                time_input_thrust = connectedNode.getCurrentTime();
                input_thrust = input_thrust_msg.getData();

                if (status_system >= 3) {
                    if (serial != null) {
                        MotorOutputs motorOutputs = new MotorOutputs(input_thrust);
                        int[] thrustVals = motorOutputs.getMotorOutputs();
                        byte[] bytes = EmbeddedManager.getEmbeddedByteArray(thrustVals, numberOfMessagesSent);
                        numberOfMessagesSent++;
                        new AsyncArduinoWrite().execute(new Object[]{serial, bytes});
                    }
                }
            }
        });
    }


    public void publishMessageData(EmbeddedManager.Message message) {
        if(!this.initialized) {
            Log.d("ERROR:", "Ros publishers were not initialized yet, so could not publish data");
            return;
        }
        status_embedded_msg.setData(message.smcStatus);
        double[] imuData = new double[4];
        for(int i = 0; i < imuData.length; i++) {
            imuData[i] =  (double)message.imuData[i] / INT_MAX;
        }
        embedded_imu_msg.setX(imuData[0]);
        embedded_imu_msg.setY(imuData[1]);
        embedded_imu_msg.setZ(imuData[2]);
        embedded_imu_msg.setW(imuData[3]);

        embedded_temperature_msg.setData(message.ambientTemp);
        double[] thrustData = new double[4];
        for(int i = 0; i < thrustData.length; i++) {
            thrustData[i] = (double)message.motorThrust[i] / INT_MAX;
        }
        embedded_thrust_msg.setData(thrustData);
        embedded_controller_states_msg.setData(message.motorStatus);
        embedded_battery_soc_msg.setData(message.battSOC);
        embedded_battery_voltage_msg.setData(message.batteryVoltage / 1000.0);

        // Publish the data
        status_embedded_pub.publish(status_embedded_msg);
        embedded_imu_pub.publish(embedded_imu_msg);
        embedded_temperature_pub.publish(embedded_temperature_msg);
        embedded_controller_states_pub.publish(embedded_controller_states_msg);
        embedded_battery_soc_pub.publish(embedded_battery_soc_msg);
        embedded_battery_voltage_pub.publish(embedded_battery_voltage_msg);

    }
}
