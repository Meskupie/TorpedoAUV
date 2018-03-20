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

import org.ros.android.view.VirtualJoystickView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.io.IOException;
import java.util.HashMap;

import Autonomy.ControllerNode;
import Autonomy.Localization.LocalizationNode;
import Autonomy.PlannerNode;
import Communication.CommunicationNode;
import Communication.MessageManager;
import Communication.SerialWrite;
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
		public enum Page_State{BEGIN_COURSE_RUN, ENABLE_RUN, WAITING_FOR_LOCK, POSITION_LOCKED, READY_TO_LAUNCH, RUNNING}        //Enum created for all page titles
		public Page_State pageState = Page_State.BEGIN_COURSE_RUN;               //state to track current page
		private Button button_nav_bottom;
		private Button button_nav_middle;
		private Button button_nav_top;
		private TextView header;
		private TextView description;
		private TextView currentDraw;
		private TextView batterySoc;
		private LinearLayout viewContainer;

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

//		text = (TextView) findViewById(R.id.textOut);
//		text.setMovementMethod(new ScrollingMovementMethod());

//			backButton = (Button) findViewById(R.id.button_nav_bottom);
//			backButton.setOnClickListener(new View.OnClickListener() {
//				@Override
//				public void onClick(View view) {
//					onBackPressed();
//				}
//			});

			//UI Stuff
			button_nav_bottom = (Button) findViewById(R.id.button_nav_bottom);
			button_nav_middle = (Button) findViewById(R.id.button_nav_middle);
			button_nav_top = (Button) findViewById(R.id.button_nav_top);
			header = (TextView) findViewById(R.id.header);
			description = (TextView) findViewById(R.id.description);
			currentDraw = (TextView) findViewById(R.id.current_info_value);
			batterySoc = (TextView) findViewById(R.id.soc_info_value);
			viewContainer = (LinearLayout) findViewById(R.id.view_container);

			onBeginCourseRun();

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
			usb_device_first_cam = new USBDeviceWrapper("First Camera",0x0525,0xa4aa);
			usb_device_first_cam.callback = usb_callback_first_cam;
			usb_device_first_cam = initializeSerial(usb_device_first_cam);

			// Setup USB Rear Cam
			usb_device_second_cam = new USBDeviceWrapper("Second Camera",0x0525,0xa4aa);
			usb_device_second_cam.callback = usb_callback_second_cam;
			usb_device_second_cam = initializeSerial(usb_device_second_cam);

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
			} catch (IOException e) {
				System.out.println("Socket error: " + e.getMessage());
			}

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
				System.out.println("Socket error: " + e.getMessage());
			}

			// Start Localization Node
			try {
				java.net.Socket socket = new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
				java.net.InetAddress local_network_address = socket.getLocalAddress();
				socket.close();
				NodeConfiguration nodeConfiguration =
						NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
				nodeMainExecutor.execute(localization_node,
						nodeConfiguration.setNodeName("rov/localization_node"));
			} catch (IOException e) {
				System.out.println("Socket error: " + e.getMessage());
			}

			// Start Planner Node
			try {
				java.net.Socket socket = new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
				java.net.InetAddress local_network_address = socket.getLocalAddress();
				socket.close();
				NodeConfiguration nodeConfiguration =
						NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
				nodeMainExecutor.execute(planner_node,
						nodeConfiguration.setNodeName("rov/planner_node"));
			} catch (IOException e) {
				System.out.println("Socket error: " + e.getMessage());
			}

			// Start Controller Node
			try {
				java.net.Socket socket = new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
				java.net.InetAddress local_network_address = socket.getLocalAddress();
				socket.close();
				NodeConfiguration nodeConfiguration =
						NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
				nodeMainExecutor.execute(controller_node,
						nodeConfiguration.setNodeName("rov/controller_node"));
			} catch (IOException e) {
				System.out.println("Socket error: " + e.getMessage());
			}

			// Start Parameters Node
			try {
				java.net.Socket socket = new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
				java.net.InetAddress local_network_address = socket.getLocalAddress();
				socket.close();
				NodeConfiguration nodeConfiguration =
						NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
				nodeMainExecutor.execute(parameters_node,
						nodeConfiguration.setNodeName("rov/parameters_node"));
			} catch (IOException e) {
				System.out.println("Socket error: " + e.getMessage());
			}

			// Set parameters
			parameters_node.setDynamics("rough_controller_data.txt");

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
		}

		// =========================== USB =================================
		private UsbSerialInterface.UsbReadCallback usb_callback_smc = new UsbSerialInterface.UsbReadCallback() {
			//Defining a Callback which triggers whenever data is read.
			@Override
			public void onReceivedData(byte[] arg0) {
				Log.d("ROV_LOG","received "+arg0.length+" bytes now");
				if (arg0.length == message_manager.msg_smc_sensors.size_bytes) {
					message_manager.msg_smc_sensors.parseData(arg0);
					communication_node.SMCSensorsPub(message_manager.msg_smc_sensors);
				} else {
					Log.d(TAG_ERROR, "First cam callback got an invalid number of bytes: " + arg0.length);
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
			this.usb_manager = (UsbManager) getSystemService(Context.USB_SERVICE);
			// TODO: change to serial number
			// Attempt to find device by vendor and product id
			HashMap<String, UsbDevice> deviceList = this.usb_manager.getDeviceList();
			for (String key : deviceList.keySet()) {
				UsbDevice device = deviceList.get(key);
				if (device.getVendorId() == device_wrapper.VENDOR_ID && device.getProductId() == device_wrapper.PRODUCT_ID) {
					device_wrapper.usbDevice = device;
					Log.d(TAG_LOG, "USB device " + device_wrapper.name + " found with serial number " + device.getSerialNumber());
					break;
				}
			}
			// setup broadcast receiver for user permission granting
			device_wrapper.usbReceiver = new BroadcastReceiver() {
				public void onReceive(Context context, Intent intent) {
					String action = intent.getAction();
					if (ACTION_USB_PERMISSION.equals(action)) {
						synchronized (this) {
							device_wrapper.usbConnection = usb_manager.openDevice(device_wrapper.usbDevice);
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
									Log.d(TAG_ERROR, "USB serial port not open for " + device_wrapper.name);
								}
							} else {
								Log.d(TAG_ERROR, "USB serial Port is null for " + device_wrapper.name);
							}
							if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
								if (device_wrapper.usbDevice != null) {
									//call method to set up device communication
								}
							} else {
								Log.d(TAG_ERROR, "USB Permission denied for " + device_wrapper.name);
							}
						}
					}
				}
			};
			// request permission from the user on create
			if (device_wrapper.usbDevice != null) {
				PendingIntent mPermissionIntent = PendingIntent.getBroadcast(this, 0, new Intent(ACTION_USB_PERMISSION), 0);
				IntentFilter filter = new IntentFilter(ACTION_USB_PERMISSION);
				this.registerReceiver(device_wrapper.usbReceiver, filter);
				this.usb_manager.requestPermission(device_wrapper.usbDevice, mPermissionIntent);
			} else {
				Log.d(TAG_ERROR, "USB could not find " + device_wrapper.name + " on startup");
			}
			return device_wrapper;
		}


		// =======================UI=============================

		private void onBeginCourseRun(){
			header.setText(R.string.begin_course_run);
			description.setText(R.string.begin_course_run_description);
			makeButtonsVisible();
			button_nav_top.setText(R.string.begin);
			button_nav_middle.setText(R.string.next);
			button_nav_bottom.setText(R.string.back);
			pageState = Page_State.BEGIN_COURSE_RUN;
            viewContainer.setBackgroundResource(R.drawable.outline_bg);

            resetButtonHandler();

            Handler handler = new Handler();
            handler.postDelayed(new Runnable() {
                @Override
                public void run() {
                    button_nav_top.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View view) {
                            onEnableRun();
                        }
                    });

                    button_nav_middle.setOnClickListener(null);

                    button_nav_bottom.setOnClickListener(null);
                }
            }, 200);
        }

		private void onEnableRun(){
			header.setText(R.string.enable_run);
			description.setText(R.string.enable_run_description);
			makeButtonsVisible();
			button_nav_top.setText(R.string.hold);
			button_nav_middle.setText(R.string.first);
			button_nav_bottom.setText(R.string.back);
			pageState = Page_State.ENABLE_RUN;
            viewContainer.setBackgroundResource(R.drawable.outline_bg);

            resetButtonHandler();

            Handler handler = new Handler();
            handler.postDelayed(new Runnable() {
                @Override
                public void run() {
                    button_nav_middle.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View view) {
                            Handler enableClick = new Handler();
                            enableClick.postDelayed(new Runnable() {
                                @Override
                                public void run() {
                                    button_nav_top.setOnTouchListener(new View.OnTouchListener() {
                                        @Override
                                        public boolean onTouch(View v, MotionEvent event) {

                                            switch (event.getAction()) {
                                                case MotionEvent.ACTION_DOWN:
                                                    break;
                                                case MotionEvent.ACTION_UP:
                                                    onEnableRun();
                                                    break;
                                            }

                                            return true;
                                        }
                                    });
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
                                            onEnableRun();
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
                }
            }, 200);


		}

        private void onWaitingForLock() {
            button_nav_top.setOnTouchListener(new View.OnTouchListener() {
                @Override
                public boolean onTouch(View v, MotionEvent event) {

                    switch (event.getAction()) {
                        case MotionEvent.ACTION_DOWN:
                            break;
                        case MotionEvent.ACTION_UP:
                            onEnableRun();
                            break;
                    }

                    return true;
                }
            });

            header.setText(R.string.waiting_for_lock);
            description.setText(R.string.waiting_for_lock_description);
            makeButtonsVisible();
            button_nav_top.setText(R.string.hold);
            button_nav_middle.setVisibility(View.INVISIBLE);
            button_nav_bottom.setVisibility(View.INVISIBLE);
            pageState = Page_State.WAITING_FOR_LOCK;
            viewContainer.setBackgroundResource(R.drawable.outline_bg);

            button_nav_bottom.setOnClickListener(null);

            //Testing to force view change
//            Handler handler2 = new Handler();
//            handler2.postDelayed(new Runnable() {
//                @Override
//                public void run() {
//                    // Do something after 5s = 5000ms
//                    onPositionLocked();
//                }
//            }, 5000);
		}

		private void onPositionLocked() {
			header.setText(R.string.position_locked);
			description.setText(R.string.position_locked_description);
			makeButtonsVisible();
			button_nav_top.setText(R.string.hold);
			button_nav_middle.setVisibility(View.INVISIBLE);
			button_nav_bottom.setVisibility(View.INVISIBLE);
			pageState = Page_State.POSITION_LOCKED;
			viewContainer.setBackgroundResource(R.drawable.outline_bg_green);

			button_nav_middle.setOnClickListener(null);

			button_nav_bottom.setOnClickListener(null);

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

//        backButton = (Button) findViewById(R.id.back_button);
//        backButton.setOnClickListener(new View.OnClickListener() {
//            @Override
//            public void onClick(View view) {
//                onBackPressed();
//            }
//        });

		private void onReadyToLaunch(){
            header.setText(R.string.ready_to_launch);
            description.setText(R.string.armed);
            makeButtonsVisible();
            button_nav_top.setText(R.string.start);
            button_nav_middle.setText(R.string.stop);
            button_nav_bottom.setText(R.string.stop);
            pageState = Page_State.READY_TO_LAUNCH;
            viewContainer.setBackgroundResource(R.drawable.outline_bg_green);

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

                    button_nav_middle.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View view) {
                            onBeginCourseRun();
                        }
                    });

                    button_nav_bottom.setOnClickListener(new View.OnClickListener() {
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
            makeButtonsVisible();
            button_nav_top.setText(R.string.stop);
            button_nav_middle.setText(R.string.stop);
            button_nav_bottom.setText(R.string.stop);
            pageState = Page_State.RUNNING;
            viewContainer.setBackgroundResource(R.drawable.outline_bg_green);

            resetButtonHandler();

            Handler handler = new Handler();
            handler.postDelayed(new Runnable() {
                @Override
                public void run() {
                    button_nav_top.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View view) {
                            onBeginCourseRun();
                        }
                    });

                    button_nav_middle.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View view) {
                            onBeginCourseRun();
                        }
                    });

                    button_nav_bottom.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View view) {
                            onBeginCourseRun();
                        }
                    });
                }
            }, 200);

        }

		private void makeButtonsVisible(){
            button_nav_top.setVisibility(View.VISIBLE);
            button_nav_middle.setVisibility(View.VISIBLE);
            button_nav_bottom.setVisibility(View.VISIBLE);
        }

        private void resetButtonHandler(){
            button_nav_top.setOnClickListener(null);
            button_nav_middle.setOnClickListener(null);
            button_nav_bottom.setOnClickListener(null);
            button_nav_top.setOnTouchListener(null);
            button_nav_middle.setOnTouchListener(null);
            button_nav_bottom.setOnTouchListener(null);
        }

		public void onClickButton1(View v) {
		}

		public void onClickButton2(View v) {
		}

		public void onClickButton3(View v) {
			new SerialWrite().execute(new Object[]{usb_device_smc.serial, message_manager.msg_smc_sensors.getRequest()});
		}

	}
