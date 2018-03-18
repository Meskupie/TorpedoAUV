package com.github.rosjava.android_apps.teleop;

import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

import com.felhr.usbserial.UsbSerialDevice;
import com.felhr.usbserial.UsbSerialInterface;
import com.github.rosjava.android_remocons.common_tools.apps.RosAppActivity;

import org.ros.android.view.VirtualJoystickView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.util.HashMap;


import Autonomy.ControllerNode;
import Autonomy.Localization.LocalizationNode;
import Autonomy.PlannerNode;
import Communication.MessageManager;
import Communication.SerialWrite;
import Communication.CommunicationNode;

import Communication.USBDeviceWrapper;

public class MainActivity extends RosAppActivity {
	public static final String TAG       = "DEBUG_MSG";
	public static final String TAG_LOG   = "ROV_LOG";
	public static final String TAG_ERROR = "ROV_ERROR";

	// USB stuff
	private UsbManager usb_manager;
	private USBDeviceWrapper usb_device_smc;
	MessageManager message_manager = new MessageManager();

	public static final String ACTION_USB_ATTACHED = "android.hardware.usb.action.USB_DEVICE_ATTACHED";
	public static final String ACTION_USB_DETACHED = "android.hardware.usb.action.USB_DEVICE_DETACHED";
	public static final String ACTION_USB_PERMISSION = "com.android.example.USB_PERMISSION";

	// ROS Nodes
	private SystemNode system_node;
	private CommunicationNode communication_node;
	private LocalizationNode localization_node;
	private PlannerNode planner_node;
	private ControllerNode controller_node;
	private ParametersNode parameters_node;

	// UI stuff
	private VirtualJoystickView virtualJoystickView;
	private Button backButton;


	public MainActivity() {
		// The RosActivity constructor configures the notification title and ticker messages.
		super("android teleop", "android teleop");
	}

	@SuppressWarnings("unchecked")
	@Override
	public void onCreate(Bundle savedInstanceState) {


		setDashboardResource(R.id.top_bar);
		setMainWindowResource(R.layout.main);
		super.onCreate(savedInstanceState);

//		text = (TextView) findViewById(R.id.textOut);
//		text.setMovementMethod(new ScrollingMovementMethod());

        backButton = (Button) findViewById(R.id.back_button);
        backButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                onBackPressed();
            }
        });

        // Setup nodes
		system_node = new SystemNode();
		communication_node = new CommunicationNode();
		localization_node = new LocalizationNode();
		planner_node = new PlannerNode();
        controller_node = new ControllerNode();
        parameters_node = new ParametersNode();

        // Setup USB SMC
		usb_device_smc = new USBDeviceWrapper("System Management Controller",0x2341,0x8036);
		usb_device_smc.callback = usb_callback_smc;
        usb_device_smc = initializeSerial(usb_device_smc);
		communication_node.setUSBSMC(usb_device_smc);

		// Setup USB Front Cam

		// Setup USB Rear Cam
	}

	private UsbSerialInterface.UsbReadCallback usb_callback_smc = new UsbSerialInterface.UsbReadCallback() {
		//Defining a Callback which triggers whenever data is read.
		@Override
		public void onReceivedData(byte[] arg0) {
			Log.d("DEBUG_MSG", "Serial found "+arg0.length+" bytes");
			String temp = "";
			for(int i = 0; i < arg0.length; i++){
				temp += arg0[i]+" ";
			}
			Log.d("DEBUG_MSG", ""+temp);
			message_manager.msg_smc_sensors.parseData(arg0);
			communication_node.SMCSensorsPub(message_manager.msg_smc_sensors);
		}
	};

	private USBDeviceWrapper initializeSerial(final USBDeviceWrapper device_wrapper) {
		this.usb_manager = (UsbManager) getSystemService(Context.USB_SERVICE);
		// TODO: change to serial number
		// Attempt to find device by vendor and product id
		HashMap<String, UsbDevice> deviceList = this.usb_manager.getDeviceList();
		for (String key: deviceList.keySet()){
			UsbDevice device = deviceList.get(key);
			if(device.getVendorId() == device_wrapper.VENDOR_ID && device.getProductId() == device_wrapper.PRODUCT_ID) {
				device_wrapper.usbDevice = device;
				Log.d(TAG_LOG, "USB device "+device_wrapper.name+" found with serial number "+device.getSerialNumber());
				break;
			}
		}
		// setup broadcast receiver for user permission granting
		device_wrapper.usbReceiver = new BroadcastReceiver() {
			public void onReceive(Context context, Intent intent) {
				String action = intent.getAction();
				if (ACTION_USB_PERMISSION.equals(action)) {
					synchronized (this) {
						device_wrapper.usbConnection = usb_manager.openDevice(device_wrapper.usbDevice );
						device_wrapper.serial = UsbSerialDevice.createUsbSerialDevice(device_wrapper.usbDevice, device_wrapper.usbConnection);
						if (device_wrapper.serial != null) {
							if (device_wrapper.serial.open()) { //Set Serial Connection Parameters.
								//serial.setDTR(true);
								device_wrapper.serial.setBaudRate(115200);
								device_wrapper.serial.setDataBits(UsbSerialInterface.DATA_BITS_8);
								device_wrapper.serial.setStopBits(UsbSerialInterface.STOP_BITS_1);
								//serial.setStopBits(UsbSerialInterface.STOP_BITS_1);
								device_wrapper.serial.setParity(UsbSerialInterface.PARITY_NONE);
								device_wrapper.serial.setFlowControl(UsbSerialInterface.FLOW_CONTROL_OFF);

								device_wrapper.serial.read(device_wrapper.callback);
							} else {
								Log.d(TAG_ERROR,"USB serial port not open for "+device_wrapper.name);
							}
						} else {
							Log.d(TAG_ERROR,"USB serial Port is null for "+device_wrapper.name);
						}
						if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
							if(device_wrapper.usbDevice != null){
								//call method to set up device communication
							}
						} else {
							Log.d(TAG_ERROR,"USB Permission denied for "+device_wrapper.name);
						}
					}
				}
			}
		};
		// request permission from the user on create
		if(device_wrapper.usbDevice != null) {
			PendingIntent mPermissionIntent = PendingIntent.getBroadcast(this, 0, new Intent(ACTION_USB_PERMISSION), 0);
			IntentFilter filter = new IntentFilter(ACTION_USB_PERMISSION);
			this.registerReceiver(device_wrapper.usbReceiver, filter);
			this.usb_manager.requestPermission(device_wrapper.usbDevice,mPermissionIntent);
		} else {
			Log.d(TAG_ERROR,"USB could not find "+device_wrapper.name+" on startup");
		}
		return device_wrapper;
	}

	public void onClickButton1(View v) {
	}

	public void onClickButton2(View v) {
	}

	public void onClickButton3(View v) {
		new SerialWrite().execute(new Object[]{usb_device_smc.serial, message_manager.msg_smc_sensors.getRequest()});
	}

	@Override
	protected void init(NodeMainExecutor nodeMainExecutor) {
		
		super.init(nodeMainExecutor);
		// Start System Node
		try {
			java.net.Socket socket = new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
			java.net.InetAddress local_network_address = socket.getLocalAddress();
			socket.close();
			NodeConfiguration nodeConfiguration =
					NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
			nodeMainExecutor.execute(system_node,
					nodeConfiguration.setNodeName("rov/system_node"));
		} catch (IOException e) {System.out.println("Socket error: " + e.getMessage());}

		// Start Communication Node
		try {
			java.net.Socket socket = new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
			java.net.InetAddress local_network_address = socket.getLocalAddress();
			socket.close();
			NodeConfiguration nodeConfiguration =
					NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
			nodeMainExecutor.execute(communication_node,
					nodeConfiguration.setNodeName("rov/communication_node"));
		} catch (IOException e) {
        	System.out.println("Socket error: " + e.getMessage());}

		// Start Localization Node
		try {
			java.net.Socket socket = new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
			java.net.InetAddress local_network_address = socket.getLocalAddress();
			socket.close();
			NodeConfiguration nodeConfiguration =
					NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
			nodeMainExecutor.execute(localization_node,
					nodeConfiguration.setNodeName("rov/localization_node"));
		} catch (IOException e) {System.out.println("Socket error: " + e.getMessage());}

		// Start Planner Node
		try {
			java.net.Socket socket = new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
			java.net.InetAddress local_network_address = socket.getLocalAddress();
			socket.close();
			NodeConfiguration nodeConfiguration =
					NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
			nodeMainExecutor.execute(planner_node,
					nodeConfiguration.setNodeName("rov/planner_node"));
		} catch (IOException e) {System.out.println("Socket error: " + e.getMessage());}

		// Start Controller Node
		try {
			java.net.Socket socket = new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
			java.net.InetAddress local_network_address = socket.getLocalAddress();
			socket.close();
			NodeConfiguration nodeConfiguration =
					NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
			nodeMainExecutor.execute(controller_node,
					nodeConfiguration.setNodeName("rov/controller_node"));
		} catch (IOException e) {System.out.println("Socket error: " + e.getMessage());}

		// Start Parameters Node
		try {
			java.net.Socket socket = new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
			java.net.InetAddress local_network_address = socket.getLocalAddress();
			socket.close();
			NodeConfiguration nodeConfiguration =
					NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
			nodeMainExecutor.execute(parameters_node,
					nodeConfiguration.setNodeName("rov/parameters_node"));
		} catch (IOException e) {System.out.println("Socket error: " + e.getMessage());}

		// Set parameters
		parameters_node.setDynamics("rough_controller_data.txt");

	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu){
		menu.add(0,0,0,R.string.stop_app);
		return super.onCreateOptionsMenu(menu);
	}

	@Override
	public boolean onOptionsItemSelected(MenuItem item){
		super.onOptionsItemSelected(item);
		switch (item.getItemId()){case 0:
			  onDestroy();
			  break;
		}
		return true;
	}

	@Override
	protected void onDestroy() {
		super.onDestroy();

		if(usb_device_smc.usbReceiver != null) {
			try {
				unregisterReceiver(usb_device_smc.usbReceiver);
				Log.d(TAG,"USB: Unregistering "+usb_device_smc.name);
			} catch (Exception e) {
				Log.d(TAG,"USB: Caught exception when unregistering "+usb_device_smc.name+": " + e.getMessage());
			}
		}
	}

}
