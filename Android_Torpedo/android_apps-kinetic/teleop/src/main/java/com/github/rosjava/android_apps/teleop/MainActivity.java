package com.github.rosjava.android_apps.teleop;

import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.os.Handler;
import android.text.method.ScrollingMovementMethod;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import com.felhr.usbserial.CDCSerialDevice;
import com.felhr.usbserial.FTDISerialDevice;
import com.felhr.usbserial.UsbSerialDevice;
import com.felhr.usbserial.UsbSerialInterface;
import com.github.rosjava.android_remocons.common_tools.apps.RosAppActivity;
import com.physicaloid.lib.Physicaloid;

import org.ros.android.view.VirtualJoystickView;
import org.ros.namespace.NameResolver;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.util.HashMap;

import Communication.AsyncArduinoWrite;
import Communication.JSONFromatter;
import Util.EmbeddedManager;
import Util.MotorOutputs;

public class MainActivity extends RosAppActivity {
	public static final String TAG = "Torpedo Debug";

	public static final String ACTION_USB_ATTACHED = "android.hardware.usb.action.USB_DEVICE_ATTACHED";
	public static final String ACTION_USB_DETACHED = "android.hardware.usb.action.USB_DEVICE_DETACHED";
	public static final String ACTION_USB_PERMISSION = "com.android.example.USB_PERMISSION";


	public static final int BOARD_VENDOR_ID = 0x2341;
	public static final int BOARD_PRODUCT_ID = 0x8036;

	public static final int TEST_VENDOR_ID = 4292;
	public static final int TEST_PRODUCT_ID = 60000;


		private VirtualJoystickView virtualJoystickView;
	private Button backButton;

	private SystemNode system_node;
	private CommunicationNode communication_node;
	private LocalizationNode localization_node;
	private PlannerNode planner_node;
	private ControllerNode controller_node;
	private ParametersNode parameters_node;

	private UsbDevice arduino;
	private UsbManager usbManager;
	private BroadcastReceiver mUsbReceiver;
	private UsbDeviceConnection usbConnection;
	private UsbSerialDevice serial;

	private TextView text;

	private boolean askingForRead = false;

	private void tvAppend(TextView tv, CharSequence text) {
		final TextView ftv = tv;
		final CharSequence ftext = text;
		runOnUiThread(new Runnable() {
			@Override public void run() {
				ftv.append(ftext); }
		});
	}


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

		text = (TextView) findViewById(R.id.textOut);
		text.setMovementMethod(new ScrollingMovementMethod());

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

        this.initializeArduino();
	}

	private void initializeArduino() {
		// Attempt to Find arduino by vendor and product id
		this.usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);

		HashMap<String, UsbDevice> deviceList = this.usbManager.getDeviceList();

		for (String key: deviceList.keySet()){
			UsbDevice device = deviceList.get(key);
			if(device.getVendorId() == BOARD_VENDOR_ID && device.getProductId() == BOARD_PRODUCT_ID) {
				this.arduino = device;
				Toast.makeText(this, "Recognized Arduino Device", Toast.LENGTH_SHORT).show();
				break;
			}
		}

		// setup broadcast receiver for user permission granting
		this.mUsbReceiver = new BroadcastReceiver() {
			public void onReceive(Context context, Intent intent) {
				String action = intent.getAction();
				if (ACTION_USB_PERMISSION.equals(action)) {
					synchronized (this) {
						usbConnection = usbManager.openDevice(arduino);
						serial = UsbSerialDevice.createUsbSerialDevice(arduino, usbConnection);
					//	serial.setDTR(true);
						if (serial != null) {

							if (serial.open()) { //Set Serial Connection Parameters.
								serial.setDTR(true);
								serial.setBaudRate(115200);
								serial.setDataBits(UsbSerialInterface.DATA_BITS_8);
								serial.setStopBits(UsbSerialInterface.STOP_BITS_1);
								serial.setStopBits(UsbSerialInterface.STOP_BITS_1);
								serial.setParity(UsbSerialInterface.PARITY_NONE);
								serial.setFlowControl(UsbSerialInterface.FLOW_CONTROL_OFF);
								serial.read(mCallback);
							} else {
								text.append("Serial Port not open!" + '\n');
							}
						} else {
							text.append("Serial Port is null" + '\n');
						}
						if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
							if(arduino != null){
								//call method to set up device communication
							}
						} else {
							text.append("Permission denied for arduino!" + '\n');
						}
					}
				}
			}
		};

		// request permission from the user on create
		if(this.arduino != null) {
			PendingIntent mPermissionIntent = PendingIntent.getBroadcast(this, 0, new Intent(ACTION_USB_PERMISSION), 0);
			IntentFilter filter = new IntentFilter(ACTION_USB_PERMISSION);
			this.registerReceiver(mUsbReceiver, filter);
			this.usbManager.requestPermission(this.arduino,mPermissionIntent);
		} else {
			Toast.makeText(this, "Could not find arduino on Startup Arduino Device", Toast.LENGTH_SHORT).show();
		}
	}

	public UsbSerialInterface.UsbReadCallback mCallback = new UsbSerialInterface.UsbReadCallback() {
		//Defining a Callback which triggers whenever data is read.
		@Override
		public void onReceivedData(byte[] arg0) {
			String data = null;
			try {
				data = new String(arg0, "UTF-8");
				tvAppend(text, "data read: " + data + '\n');
				tvAppend(text, "" + '\n');

				EmbeddedManager manager = new EmbeddedManager();
				EmbeddedManager.Message message = manager.parseBytes(arg0);
				System.out.println(message.toString());
				tvAppend(text, "Converted Message: " + 'n' + message.toString());

			} catch (UnsupportedEncodingException e) {
				e.printStackTrace();
			}
		}
	};


	public void onClickButton1(View v) {
		// read
		//new AsyncArduinoWrite().execute(new Object[] {serial, "q"});
		//set boolean flag to read
		this.askingForRead = true;
		text.clearComposingText();
	}

	public void onClickButton2(View v) {
//		EmbeddedManager manager = new EmbeddedManager();
//		byte[] test = manager.hexStringToByteArray("D930F7EA2183EF009F120000000000000000000000000000FFBC1F33333300");
//		EmbeddedManager.Message message = manager.parseBytes(test);
//		System.out.println(message.toString());
		initializeArduino();
	}


	public void onClickButton3(View v) {
		new AsyncArduinoWrite().execute(new Object[]{serial,  "q"});

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

		if(this.mUsbReceiver != null) {
			try {
				unregisterReceiver(this.mUsbReceiver);
				textAppend("Unregistered usb receiver");

			} catch (Exception e) {
				textAppend("Caught exception: " + e.getMessage());
			}
		}
	}

	public void textAppend(String str) {
		text.append(str + '\n');
	}

}
