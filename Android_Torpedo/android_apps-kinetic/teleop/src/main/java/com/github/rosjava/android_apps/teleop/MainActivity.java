package com.github.rosjava.android_apps.teleop;

import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.InputDevice;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;

import com.felhr.usbserial.UsbSerialDevice;
import com.felhr.usbserial.UsbSerialInterface;
import com.github.rosjava.android_remocons.common_tools.apps.RosAppActivity;

import org.ejml.simple.SimpleMatrix;
import org.ros.android.view.VirtualJoystickView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import Autonomy.ControllerNode;
import Autonomy.Localization.LocalizationNode;
import Autonomy.PlannerNode;
import Communication.CommunicationNode;
import Communication.MessageManager;
import Communication.ReedSwitchManager;
import Communication.USBDeviceWrapper;

	public class MainActivity extends RosAppActivity {
		public static final String TAG = "DEBUG_MSG";
		public static final String TAG_LOG = "ROV_LOG";
		public static final String TAG_ERROR = "ROV_ERROR";

		// USB stuff
		private UsbManager usb_manager;
		private int[] usb_ids = new int[5];
		private int usb_number_registered = 0;
		private USBDeviceWrapper usb_device_smc;
		private USBDeviceWrapper usb_device_first_cam;
		private USBDeviceWrapper usb_device_second_cam;
		MessageManager message_manager = new MessageManager();

		// ROS Nodes
		private SystemNode system_node;
		private CommunicationNode communication_node;
		private LocalizationNode localization_node;
		private PlannerNode planner_node;
		private ControllerNode controller_node;
		private ParametersNode parameters_node;

		// UI stuff
		private VirtualJoystickView virtualJoystickView;
		public enum page_state {BEGIN_COURSE_RUN, ENABLE_RUN, WAITING_FOR_LOCK, POSITION_LOCKED, READY_TO_LAUNCH, RUNNING}        //Enum created for all page titles
		public page_state pageState = page_state.BEGIN_COURSE_RUN;               //state to track current page
		private Button button_nav_bottom;
		private Button button_nav_middle;
		private Button button_nav_top;
		private TextView header;
		private TextView description;
		private LinearLayout viewContainer;
		private Handler monitorUIHandler;
		private TextView translation_target;
		private TextView rotation_target;

		private TextView currentDraw;
		private TextView batterySoc;
		private TextView runState;
		private String[] run_states = new String[]{"Error","Init","Idle","Lock","Arm","Hold","Run"};

		private ReedSwitchManager frontReedSwitch;
		private ReedSwitchManager centerReedSwitch;
		private ReedSwitchManager rearReedSwitch;

		public MainActivity(){
			// The RosActivity constructor configures the notification title and ticker messages.
			super("android teleop", "android teleop");
		}

		@SuppressWarnings("unchecked")
		@Override
		public void onCreate(Bundle savedInstanceState) {

			setDashboardResource(R.id.top_bar);
			setMainWindowResource(R.layout.main);
			super.onCreate(savedInstanceState);

			// Setup Reed switch manager
			this.frontReedSwitch = new ReedSwitchManager(button_nav_top);
			this.centerReedSwitch = new ReedSwitchManager(button_nav_middle);
			this.rearReedSwitch = new ReedSwitchManager(button_nav_bottom);

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

//			// Setup USB Front Cam
//			usb_device_first_cam = new USBDeviceWrapper("First Camera",0x0525,0xa4aa);
//			usb_device_first_cam.callback = usb_callback_first_cam;
//			usb_device_first_cam = initializeSerial(usb_device_first_cam);
//
//			// Setup USB Rear Cam
//			usb_device_second_cam = new USBDeviceWrapper("Second Camera",0x0525,0xa4aa);
//			usb_device_second_cam.callback = usb_callback_second_cam;
//			usb_device_second_cam = initializeSerial(usb_device_second_cam);

			// Setup Game Controller
			ArrayList controllerIds = getGameControllerIds();
			if(controllerIds.size() == 0){
				Log.d("ROV_ERROR", "Could not find joystick device");
			}else if(controllerIds.size() == 1){
				Log.d(TAG_LOG, "Input device found with id: "+controllerIds.get(0));
			}else{
				Log.d("ROV_ERROR", "Found too many input devices");
			}

			//UI Stuff
			button_nav_bottom = (Button) findViewById(R.id.button_nav_bottom);
			button_nav_middle = (Button) findViewById(R.id.button_nav_middle);
			button_nav_top = (Button) findViewById(R.id.button_nav_top);
			header = (TextView) findViewById(R.id.header);
			description = (TextView) findViewById(R.id.description);
			currentDraw = (TextView) findViewById(R.id.current_info_value);
			batterySoc = (TextView) findViewById(R.id.soc_info_value);
			runState = (TextView) findViewById(R.id.run_state);
			viewContainer = (LinearLayout) findViewById(R.id.view_container);
			translation_target = (TextView) findViewById(R.id.translation_target);
			rotation_target = (TextView) findViewById(R.id.rotation_target);
			//UI updater
			monitorUIHandler = new Handler();
			monitorUI.run();

			onBeginCourseRun();
		}


		@Override
		protected void init(NodeMainExecutor nodeMainExecutor) {

			super.init(nodeMainExecutor);
			// Start System Node
			try {
				java.net.Socket socket = new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
				java.net.InetAddress local_network_address = socket.getLocalAddress();
				socket.close();
				NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());

				// Start all nodes
				nodeMainExecutor.execute(system_node, nodeConfiguration.setNodeName("rov/system_node"));
				nodeMainExecutor.execute(communication_node, nodeConfiguration.setNodeName("rov/communication_node"));
				nodeMainExecutor.execute(localization_node,	nodeConfiguration.setNodeName("rov/localization_node"));
				nodeMainExecutor.execute(planner_node, nodeConfiguration.setNodeName("rov/planner_node"));
				nodeMainExecutor.execute(controller_node, nodeConfiguration.setNodeName("rov/controller_node"));
				nodeMainExecutor.execute(parameters_node, nodeConfiguration.setNodeName("rov/parameters_node"));
			} catch (IOException e) {
				System.out.println("Socket error: " + e.getMessage());
			}

			// Set parameters
			parameters_node.setDynamics("rough_controller_2.txt");
			parameters_node.setRunMode(2);
			parameters_node.setTeleopStyle(3);
			parameters_node.setInitialPose(new Transform(new Vector3(0,0,0.05),new Quaternion(0,0,0,1)));
		}

		@Override
		public boolean onCreateOptionsMenu(Menu menu) {
			menu.add(0, 0, 0, R.string.stop_app);
			return super.onCreateOptionsMenu(menu);
		}

		@Override
		public boolean onOptionsItemSelected(MenuItem item) {
			super.onOptionsItemSelected(item);
			switch (item.getItemId()) {
				case 0:
					onDestroy();
					break;
			}
			return true;
		}

		@Override
		protected void onDestroy() {
			super.onDestroy();

			if (usb_device_smc.usbReceiver != null) {
				try {
					unregisterReceiver(usb_device_smc.usbReceiver);
					Log.d(TAG, "USB: Unregistering " + usb_device_smc.name);
				} catch (Exception e) {
					Log.d(TAG, "USB: Caught exception when unregistering " + usb_device_smc.name + ": " + e.getMessage());
				}
			}

			monitorUIHandler.removeCallbacks(monitorUI);
		}

		// =========================== USB =================================
		private UsbSerialInterface.UsbReadCallback usb_callback_smc = new UsbSerialInterface.UsbReadCallback() {
			//Defining a Callback which triggers whenever data is read.
			@Override
			public void onReceivedData(byte[] arg0) {
				Log.d("ROV_SERIAL","received "+arg0.length+" bytes now");
				if (arg0.length == message_manager.msg_smc_sensors.size_bytes) {
					message_manager.msg_smc_sensors.parseData(arg0);
					communication_node.SMCSensorsPub(message_manager.msg_smc_sensors);
					// update switches
					MainActivity.this.runOnUiThread(new Runnable() {
						public void run() {
							frontReedSwitch.updateSwitchStates(message_manager.msg_smc_sensors.switch_front);
							centerReedSwitch.updateSwitchStates(message_manager.msg_smc_sensors.switch_center);
							rearReedSwitch.updateSwitchStates(message_manager.msg_smc_sensors.switch_rear);
						}
					});
				} else {
					Log.d(TAG_ERROR, "SMC callback got an invalid number of bytes: " + arg0.length);
				}
			}
		};

		private UsbSerialInterface.UsbReadCallback usb_callback_first_cam = new UsbSerialInterface.UsbReadCallback() {
			@Override
			public void onReceivedData(byte[] arg0) {
				if(arg0.length == 1){ // This tells us what device it is
					if(arg0[0] == 0x66) { //'f' this tells us its a front camera
						usb_device_first_cam.name = "Front Camera";
						communication_node.setUSBFrontCam(usb_device_first_cam);
					}else if(arg0[0] == 0x72) { //'r' this tells us its a rear camera
						usb_device_first_cam.name = "Rear Camera";
						communication_node.setUSBRearCam(usb_device_first_cam);
					}else{ // ignore this
						Log.d(TAG_ERROR,"First cam ("+usb_device_first_cam.name+") callback got an invalid byte");
					}
				}else if(arg0.length == message_manager.msg_front_targets.size_bytes){
					if(usb_device_first_cam.name == "Front Camera"){
						message_manager.msg_front_targets.parseData(arg0);
						communication_node.frontCameraPub(message_manager.msg_front_targets);
					}else if(usb_device_first_cam.name == "Rear Camera") {
						message_manager.msg_rear_targets.parseData(arg0);
						communication_node.rearCameraPub(message_manager.msg_rear_targets);
					}else{
						Log.d(TAG_ERROR,"Unrecognized name in first camera callback: "+usb_device_first_cam.name);
					}
				}else{
					Log.d(TAG_ERROR,"First cam ("+usb_device_first_cam.name+") callback got an invalid number of bytes: "+arg0.length);
				}
			}
		};

		private UsbSerialInterface.UsbReadCallback usb_callback_second_cam = new UsbSerialInterface.UsbReadCallback() {
			@Override
			public void onReceivedData(byte[] arg0) {
				if(arg0.length == 1){ // This tells us what device it is
					if(arg0[0] == 0x66) { // this tells us its a front camera
						usb_device_second_cam.name = "Front Camera";
						communication_node.setUSBFrontCam(usb_device_second_cam);
					}else if(arg0[0] == 0x72) { // this tells us its a rear camera
						usb_device_second_cam.name = "Rear Camera";
						communication_node.setUSBRearCam(usb_device_second_cam);
					}else{ // ignore this
						Log.d(TAG_ERROR,"Second cam ("+usb_device_second_cam.name+") callback got an invalid byte");
					}
				}else if(arg0.length == message_manager.msg_front_targets.size_bytes){
					if(usb_device_second_cam.name == "Front Camera"){
						message_manager.msg_front_targets.parseData(arg0);
						communication_node.frontCameraPub(message_manager.msg_front_targets);
					}else if(usb_device_second_cam.name == "Rear Camera") {
						message_manager.msg_rear_targets.parseData(arg0);
						communication_node.rearCameraPub(message_manager.msg_rear_targets);
					}else{
						Log.d(TAG_ERROR,"Unrecognized name in first camera callback: "+usb_device_first_cam.name);
					}
				}else{
					Log.d(TAG_ERROR,"Second cam ("+usb_device_second_cam.name+") callback got an invalid number of bytes: "+arg0.length);
				}
			}
		};

		private USBDeviceWrapper initializeSerial(final USBDeviceWrapper device_wrapper) {
			final String ACTION_USB_ATTACHED = "android.hardware.usb.action.USB_DEVICE_ATTACHED";
			final String ACTION_USB_DETACHED = "android.hardware.usb.action.USB_DEVICE_DETACHED";
			final String ACTION_USB_PERMISSION = "com.android.example.USB_PERMISSION";
			this.usb_manager = (UsbManager) getSystemService(Context.USB_SERVICE);
			// TODO: change to serial number
			// Attempt to find device by vendor and product id
			HashMap<String, UsbDevice> deviceList = this.usb_manager.getDeviceList();
			for (String key: deviceList.keySet()){
				UsbDevice device = deviceList.get(key);
				if(device.getVendorId() == device_wrapper.VENDOR_ID && device.getProductId() == device_wrapper.PRODUCT_ID) {
					boolean seen = false;
					int id = device.getDeviceId();
					for(int i = 0; i < (usb_number_registered); i++){
						if(id == usb_ids[i]){seen = true;}
					}
					if(!seen){
						usb_ids[usb_number_registered] = id;
						usb_number_registered++;
						device_wrapper.usbDevice = device;
						Log.d(TAG_LOG, "USB device " + device_wrapper.name + " found with id: " + id);
						break;
					}
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
									//device_wrapper.serial.setDTR(true);
									device_wrapper.serial.setBaudRate(115200);
									device_wrapper.serial.setDataBits(UsbSerialInterface.DATA_BITS_8);
									device_wrapper.serial.setStopBits(UsbSerialInterface.STOP_BITS_1);
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

		// ================Control Device========================

		public ArrayList getGameControllerIds() {
			ArrayList gameControllerDeviceIds = new ArrayList();
			int[] deviceIds = InputDevice.getDeviceIds();
			for (int deviceId : deviceIds) {
				InputDevice dev = InputDevice.getDevice(deviceId);
				int sources = dev.getSources();


				// Verify that the device has gamepad buttons, control sticks, or both.
				boolean has_gamepad = ((sources & InputDevice.SOURCE_GAMEPAD) == InputDevice.SOURCE_GAMEPAD);
				boolean has_joystick = ((sources & InputDevice.SOURCE_JOYSTICK) == InputDevice.SOURCE_JOYSTICK);
				if (has_gamepad||has_joystick) {
					// This device is a game controller. Store its device ID.
					if (!gameControllerDeviceIds.contains(deviceId)) {
						gameControllerDeviceIds.add(deviceId);
					}
				}
			}
			return gameControllerDeviceIds;
		}

		@Override
		public boolean dispatchGenericMotionEvent(final MotionEvent event) {
			// Check that the event came from a game controller
			// Log.d("ROV_STATE","event");
			if ((event.getSource() & InputDevice.SOURCE_JOYSTICK) == InputDevice.SOURCE_JOYSTICK
					&& event.getAction() == MotionEvent.ACTION_MOVE) {
//					InputDevice mInputDevice = event.getDevice();it l
//					mInputDevice.getMotionRange(MotionEvent.AXIS_X, event.getSource()).getRange();

				SimpleMatrix joy_state = new SimpleMatrix(6,1);
				joy_state.set(0,0,(-1)*event.getAxisValue(MotionEvent.AXIS_Y));
				joy_state.set(1,0,( 1)*event.getAxisValue(MotionEvent.AXIS_X));
				joy_state.set(2,0,( 1)*event.getAxisValue(MotionEvent.AXIS_Z));
				joy_state.set(3,0,(-1)*event.getAxisValue(MotionEvent.AXIS_RY));
				joy_state.set(4,0,( 1)*event.getAxisValue(MotionEvent.AXIS_RX));
				joy_state.set(5,0,( 1)*event.getAxisValue(MotionEvent.AXIS_RZ));

				//Log.d("ROV_JOY",String.format("X:%2.2f  Y:%2.2f  Z:%2.2f  R:%1.3f  P:%1.3f  W:%1.3f",joy_state.get(0),joy_state.get(1),joy_state.get(2),joy_state.get(3),joy_state.get(4),joy_state.get(5)));
				planner_node.setJoystickInput(joy_state);

				return true;
			}
			return MainActivity.super.onGenericMotionEvent(event);
		}

		// =======================UI=============================

		private void onBeginCourseRun(){
			header.setText(R.string.begin_course_run);
			description.setText(R.string.begin_course_run_description);
			makeVisible();
			button_nav_top.setText(R.string.begin);
			button_nav_middle.setText(R.string.next);
			button_nav_bottom.setText(R.string.back);
			pageState = page_state.BEGIN_COURSE_RUN;
            viewContainer.setBackgroundResource(R.drawable.outline_bg);
            translation_target.setVisibility(View.INVISIBLE);
            rotation_target.setVisibility(View.INVISIBLE);
            try {
				system_node.setDesiredState(1);
			}catch(Exception e){}

            resetButtonHandler();

            Handler handler = new Handler();
            handler.postDelayed(new Runnable() {
                @Override
                public void run() {
                    button_nav_top.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View view) {
                        	if(system_node.status_system > 0) {
								onEnableRun();
							}else{
                        		onBeginCourseRun();
							}
                        }
                    });
                    frontReedSwitch.setOnClickListener(new ReedSwitchManager.OnClickListener() {
						@Override
						public void onClick(View view) {
							if(system_node.status_system > 0) {
								onEnableRun();
							}else{
								onBeginCourseRun();
							}
						}
					});

                    button_nav_middle.setOnClickListener(null);
                    centerReedSwitch.setOnClickListener(null);

                    button_nav_bottom.setOnClickListener(null);
                    rearReedSwitch.setOnClickListener(null);
                }
            }, 200);
        }

		private void onEnableRun(){
			header.setText(R.string.enable_run);
			description.setText(R.string.enable_run_description);
			makeVisible();
			button_nav_top.setText(R.string.hold);
			button_nav_middle.setText(R.string.first);
			button_nav_bottom.setText(R.string.back);
			pageState = page_state.ENABLE_RUN;
            viewContainer.setBackgroundResource(R.drawable.outline_bg);
			translation_target.setVisibility(View.INVISIBLE);
			rotation_target.setVisibility(View.INVISIBLE);
			system_node.setDesiredState(1);

            resetButtonHandler();

            Handler handler = new Handler();
            handler.postDelayed(new Runnable() {

            	// HANDLER FOR BUTTON CLICKS
                @Override
                public void run() {
                    button_nav_middle.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View view) {
                            Handler enableClick = new Handler();
                            enableClick.postDelayed(new Runnable() {
                                @Override
                                public void run() {
									if(pageState == page_state.ENABLE_RUN){
										button_nav_top.setOnTouchListener(new View.OnTouchListener() {
											@Override
											public boolean onTouch(View v, MotionEvent event) {
												switch (event.getAction()) {
													case MotionEvent.ACTION_DOWN:
														break;
													case MotionEvent.ACTION_UP:
														onBeginCourseRun();
														break;
												}

												return true;
											}
										});
									}
                                }
                            }, 2000);

                            button_nav_top.setOnTouchListener(new View.OnTouchListener() {
                                @Override
                                public boolean onTouch(View v, MotionEvent event) {

                                    switch (event.getAction()) {
                                        case MotionEvent.ACTION_DOWN:
                                            onWaitingForLock();
                                            break;
                                        case MotionEvent.ACTION_UP:
											if(pageState == page_state.ENABLE_RUN){
												onBeginCourseRun();
											}
                                            break;
                                    }

                                    return true;
                                }
                            });
                        }
                    });

					button_nav_bottom.setOnClickListener(new View.OnClickListener() {
						@Override
						public void onClick(View view) {
							onBeginCourseRun();
						}
					});

                    //SAME HANDLER FOR REED SWITCHES
					centerReedSwitch.setOnClickListener(new ReedSwitchManager.OnClickListener() {
						@Override
						public void onClick(View view) {
							Handler enableClick = new Handler();
							enableClick.postDelayed(new Runnable() {
								@Override
								public void run() {
									if(pageState == page_state.ENABLE_RUN) {
										frontReedSwitch.setOnTouchListener(new ReedSwitchManager.OnTouchListener() {
											@Override
											public void onTouch(View v, MotionEvent event) {
												switch (event.getAction()) {
													case MotionEvent.ACTION_DOWN:
														break;
													case MotionEvent.ACTION_UP:
														onBeginCourseRun();
														break;
												}
											}
										});
									}
								}
							}, 2000);

							frontReedSwitch.setOnTouchListener(new ReedSwitchManager.OnTouchListener() {
								@Override
								public void onTouch(View v, MotionEvent event) {
									switch (event.getAction()) {
										case MotionEvent.ACTION_DOWN:
											onWaitingForLock();
											break;
										case MotionEvent.ACTION_UP:
											if(pageState == page_state.ENABLE_RUN){
												onBeginCourseRun();
											}
											break;
									}
								}
							});
						}
					});

                    rearReedSwitch.setOnClickListener(new ReedSwitchManager.OnClickListener() {
                        @Override
                        public void onClick(View view) {
                            onBeginCourseRun();
                        }
                    });
                }
            }, 200);
		}

        private void onWaitingForLock() {
			header.setText(R.string.waiting_for_lock);
			description.setText(R.string.waiting_for_lock_description);
			makeVisible();
			button_nav_top.setText(R.string.hold);
			button_nav_middle.setVisibility(View.INVISIBLE);
			button_nav_bottom.setVisibility(View.INVISIBLE);
			pageState = page_state.WAITING_FOR_LOCK;
			viewContainer.setBackgroundResource(R.drawable.outline_bg);
			translation_target.setVisibility(View.INVISIBLE);
			rotation_target.setVisibility(View.INVISIBLE);
			system_node.setDesiredState(2);

			button_nav_bottom.setOnClickListener(null);
			rearReedSwitch.setOnClickListener(null);

            button_nav_top.setOnTouchListener(new View.OnTouchListener() {
				@Override
				public boolean onTouch(View v, MotionEvent event) {

					switch (event.getAction()) {
						case MotionEvent.ACTION_DOWN:
							break;
						case MotionEvent.ACTION_UP:
							onBeginCourseRun();
							break;
					}

					return true;
				}
			});

			frontReedSwitch.setOnTouchListener(new ReedSwitchManager.OnTouchListener() {
				@Override
				public void onTouch(View v, MotionEvent event) {

					switch (event.getAction()) {
						case MotionEvent.ACTION_DOWN:
							break;
						case MotionEvent.ACTION_UP:
							onBeginCourseRun();
							break;
					}
				}
			});
            Handler handler2 = new Handler();
            handler2.postDelayed(new Runnable() {
                @Override
                public void run() {
                    if(system_node.isStateTransitionReady()){
						onPositionLocked();
					}else{
                    	Log.d("ROV_ERROR", "Position lock was not true after timeout");
                    	onBeginCourseRun();
					}
                }
            }, 1000);
		}

		private void onPositionLocked() {
			header.setText(R.string.position_locked);
			description.setText(R.string.position_locked_description);
			makeVisible();
			button_nav_top.setText(R.string.hold);
			button_nav_middle.setVisibility(View.INVISIBLE);
			button_nav_bottom.setVisibility(View.INVISIBLE);
			pageState = page_state.POSITION_LOCKED;
			viewContainer.setBackgroundResource(R.drawable.outline_bg);
			translation_target.setVisibility(View.VISIBLE);
			rotation_target.setVisibility(View.VISIBLE);
			system_node.setDesiredState(3);

			button_nav_top.setOnTouchListener(new View.OnTouchListener() {
				@Override
				public boolean onTouch(View v, MotionEvent event) {

					switch (event.getAction()) {
						case MotionEvent.ACTION_DOWN:
							break;
						case MotionEvent.ACTION_UP:
							if(system_node.isStateTransitionReady()){
								onReadyToLaunch();
							}else{
								Log.d(TAG_LOG, "Attempted arm at too far a distance from start");
								onBeginCourseRun();
							}
							break;
					}

					return true;
				}
			});

			frontReedSwitch.setOnTouchListener(new ReedSwitchManager.OnTouchListener() {
				@Override
				public void onTouch(View v, MotionEvent event) {
					switch (event.getAction()) {
						case MotionEvent.ACTION_DOWN:
							break;
						case MotionEvent.ACTION_UP:
							if(system_node.isStateTransitionReady()){
								onReadyToLaunch();
							}else{
								Log.d(TAG_LOG, "Attempted arm at too far a distance from start");
								onBeginCourseRun();
							}
							break;
					}
				}
			});

			button_nav_middle.setOnClickListener(null);
			button_nav_bottom.setOnClickListener(null);

			centerReedSwitch.setOnClickListener(null);
			rearReedSwitch.setOnClickListener(null);

			//Testing to force view change
//            Handler handler = new Handler();
//            handler.postDelayed(new Runnable() {
//                @Override
//                public void run() {
//                    // Do something after 5s = 5000ms
//                    onReadyToLaunch();
//                }
//            }, 5000);
		}

		private void onReadyToLaunch(){
            header.setText(R.string.ready_to_launch);
            description.setText(R.string.armed);
            makeVisible();
            button_nav_top.setText(R.string.start);
            button_nav_middle.setText(R.string.stop);
            button_nav_bottom.setText(R.string.stop);
            pageState = page_state.READY_TO_LAUNCH;
            viewContainer.setBackgroundResource(R.drawable.outline_bg_green);
			translation_target.setVisibility(View.INVISIBLE);
			rotation_target.setVisibility(View.INVISIBLE);
			system_node.setDesiredState(4);

            resetButtonHandler();

            Handler handler = new Handler();
            handler.postDelayed(new Runnable() {
                @Override
                public void run() {
                    button_nav_top.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View view) {
                            onRunning();
                            resetButtonHandler();
                        }
                    });
					frontReedSwitch.setOnClickListener(new ReedSwitchManager.OnClickListener() {
						@Override
						public void onClick(View view) {
							onRunning();
							resetButtonHandler();
						}
					});

                    button_nav_middle.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View view) {onBeginCourseRun(); }
                    });
					centerReedSwitch.setOnClickListener(new ReedSwitchManager.OnClickListener() {
						@Override
						public void onClick(View view) {
							onBeginCourseRun();
						}
					});

                    button_nav_bottom.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View view) {onBeginCourseRun();
                        }
                    });
					rearReedSwitch.setOnClickListener(new ReedSwitchManager.OnClickListener() {
						@Override
						public void onClick(View view) {
							onBeginCourseRun();
						}
					});
                }
            }, 200);
        }

		private void onRunning(){
            header.setText(R.string.running);
            description.setText(null);
            makeVisible();
            button_nav_top.setText(R.string.stop);
            button_nav_middle.setText(R.string.stop);
            button_nav_bottom.setText(R.string.stop);
			description.setVisibility(View.INVISIBLE);
            pageState = page_state.RUNNING;
            viewContainer.setBackgroundResource(R.drawable.outline_bg_green);
			translation_target.setVisibility(View.INVISIBLE);
			rotation_target.setVisibility(View.INVISIBLE);
			system_node.setDesiredState(5);

            resetButtonHandler();

            Handler handler = new Handler();
            handler.postDelayed(new Runnable() {
                @Override
                public void run() {
                    button_nav_top.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View view) {onBeginCourseRun();
                        }
                    });
					frontReedSwitch.setOnClickListener(new ReedSwitchManager.OnClickListener() {
						@Override
						public void onClick(View view) {
							onBeginCourseRun();
						}
					});

                    button_nav_middle.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View view) {onBeginCourseRun();
                        }
                    });
					centerReedSwitch.setOnClickListener(new ReedSwitchManager.OnClickListener() {
						@Override
						public void onClick(View view) {
							onBeginCourseRun();
						}
					});

                    button_nav_bottom.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View view) {onBeginCourseRun();
                        }
                    });
					rearReedSwitch.setOnClickListener(new ReedSwitchManager.OnClickListener() {
						@Override
						public void onClick(View view) {
							onBeginCourseRun();
						}
					});
                }
            }, 200);

        }

		private void makeVisible(){
            button_nav_top.setVisibility(View.VISIBLE);
            button_nav_middle.setVisibility(View.VISIBLE);
            button_nav_bottom.setVisibility(View.VISIBLE);
			description.setVisibility(View.VISIBLE);
        }

        private void resetButtonHandler(){
            button_nav_top.setOnClickListener(null);
            button_nav_middle.setOnClickListener(null);
            button_nav_bottom.setOnClickListener(null);
            button_nav_top.setOnTouchListener(null);
            button_nav_middle.setOnTouchListener(null);
            button_nav_bottom.setOnTouchListener(null);

			frontReedSwitch.setOnClickListener(null);
			centerReedSwitch.setOnClickListener(null);
			rearReedSwitch.setOnClickListener(null);
			frontReedSwitch.setOnTouchListener(null);
			centerReedSwitch.setOnTouchListener(null);
			rearReedSwitch.setOnTouchListener(null);
        }

		Runnable monitorUI = new Runnable() {
			@Override
			public void run() {
				try {
					//currentDraw;
					batterySoc.setText(String.valueOf(((double)((int)(communication_node.ui_battery_voltage*100)))/100).concat("V"));
					runState.setText(run_states[system_node.status_system+1]);
					if(system_node.status_system >= 1){
						runState.setBackgroundResource(R.drawable.outline_bg_green);
					}else if(system_node.status_system == 0){
						runState.setBackgroundResource(R.drawable.outline_bg_blue);
					}else{
						runState.setBackgroundResource(R.drawable.outline_bg_red);
					}
					Vector3 trans_targ = planner_node.rov_planner.translation_initial_error;
					Vector3 rot_targ = planner_node.rov_planner.rotation_initial_error;
					//Log.d(TAG,String.format("Dist to target  X:%2.2f  Y:%2.2f  Z:%2.2f",trans_targ.getX(),trans_targ.getY(),trans_targ.getZ()));
					//Log.d(TAG,String.format("Dist to target  R:"+rot_targ.getX()+"  P:"+rot_targ.getY()+"  W:"+rot_targ.getZ()));
					translation_target.setText(String.format("Dist to target  X:%1.3f  Y:%1.3f  Z:%1.3f",trans_targ.getX(),trans_targ.getY(),trans_targ.getZ()));
					rotation_target.setText(String.format("Dist to target  R:%1.3f  P:%1.3f  W:%1.3f",rot_targ.getX(),rot_targ.getY(),rot_targ.getZ()));
					if(planner_node.rov_planner.ready_initial_translation){
						translation_target.setBackgroundResource(R.drawable.outline_bg_green);
					}else{
						translation_target.setBackgroundResource(R.drawable.outline_bg_red);
					}
					if(planner_node.rov_planner.ready_initial_rotation){
						rotation_target.setBackgroundResource(R.drawable.outline_bg_green);
					}else{
						rotation_target.setBackgroundResource(R.drawable.outline_bg_red);
					}
				} finally {
					monitorUIHandler.postDelayed(monitorUI, 50);
				}
			}
		};

	}
