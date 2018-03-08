package com.github.rosjava.android_apps.teleop;

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

import com.github.rosjava.android_remocons.common_tools.apps.RosAppActivity;
import com.physicaloid.lib.Physicaloid;

import org.ros.android.view.VirtualJoystickView;
import org.ros.namespace.NameResolver;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.io.IOException;

import Communication.JSONFromatter;
import Util.MotorOutputs;

public class MainActivity extends RosAppActivity {
	private VirtualJoystickView virtualJoystickView;
	private Button backButton;

	private SystemNode system_node;
	private CommunicationNode communication_node;
	private LocalizationNode localization_node;
	private PlannerNode planner_node;
	private ControllerNode controller_node;
	private ParametersNode parameters_node;


	private TextView text;

	private EditText FL;
	private EditText FR;
	private EditText FV;
	private EditText BL;
	private EditText BR;
	private EditText BV;

	private Physicaloid physicaloid;

	Handler mHandler = new Handler();
	Handler mHandler2 = new Handler();

	private void tvAppend(TextView tv, String text, boolean fromAndroid) {
		final TextView ftv = tv;
		String append  = fromAndroid ? "Android: " : "Atmega";

		//final CharSequence ftext = append + text + '\n';
		final CharSequence ftext = (fromAndroid?'\n':"") + text;

		mHandler.post(new Runnable() {
			@Override
			public void run() {
				ftv.append(ftext); 	// add text to Text view
			}
		});
	}

	private void tvClear(TextView tv) {
		final TextView ftv = tv;
		mHandler2.post(new Runnable() {
			@Override
			public void run() {
				ftv.setText(""); // clear text
			}
		});
	}

	final Handler timeDelayHandler = new Handler();



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


		try {

			physicaloid = new Physicaloid(this);
			physicaloid.setBaudrate(9600);

		} catch (Exception e) {
			Log.d("IO EXCEPTION", "there was a serial exception");
		}

		FL = (EditText) findViewById(R.id.FL);
		FR = (EditText) findViewById(R.id.FR);
		FV = (EditText) findViewById(R.id.FV);
		BL = (EditText) findViewById(R.id.BL);
		BR = (EditText) findViewById(R.id.BR);
		BV = (EditText) findViewById(R.id.BV);

		text = (TextView) findViewById(R.id.textOut);
		text.setMovementMethod(new ScrollingMovementMethod());

		virtualJoystickView = (VirtualJoystickView) findViewById(R.id.virtual_joystick);

        backButton = (Button) findViewById(R.id.back_button);
        backButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                onBackPressed();
            }
        });
		doRoutine();


        // Setup nodes
		system_node = new SystemNode();
		communication_node = new CommunicationNode();
		localization_node = new LocalizationNode();
		planner_node = new PlannerNode();
        controller_node = new ControllerNode();
        parameters_node = new ParametersNode();
	}

	private void delayedCallToSetMotors(int millis, final MotorOutputs outputs) {
		timeDelayHandler.postDelayed(new Runnable() {
			@Override
			public void run() {
				updateMotorDisplay(outputs);
				setOutputsToMotors(outputs);
			}
		}, millis);
	}


	public void beginRoutine(View v) {
		doRoutine();
	}

	public void doRoutine() {
		delayedCallToSetMotors(0000, getMotorOutputs(0.1, 0.1));
		delayedCallToSetMotors(5000, getMotorOutputs(-0.0, 0.1));
		delayedCallToSetMotors(12000, getMotorOutputs(0.1, 0.1));
		delayedCallToSetMotors(16000, getMotorOutputs(0.0, -0.1));
		delayedCallToSetMotors(20000, getMotorOutputs(-0.1, -0.1));
	}

	public MotorOutputs getMotorOutputs(double x, double y){
		return new MotorOutputs(new double[]{-x+y, -x+y, 0, x+y, x+y, 0});
	}

	public void updateMotors(double x, double y) {
		MotorOutputs outputs = new MotorOutputs(new double[]{-x+y, -x+y, 0, x+y, x+y, 0});
		updateMotorDisplay(outputs);
		setOutputsToMotors(outputs);
	}
	public void updateMotorDisplay(MotorOutputs outputs) {

		FL.setText(String.format("%.3f", outputs.getNormalized(MotorOutputs.Motor.FL)));
		FR.setText(String.format("%.3f", outputs.getNormalized(MotorOutputs.Motor.FR)));
		FV.setText(String.format("%.3f", outputs.getNormalized(MotorOutputs.Motor.FV)));
		BL.setText(String.format("%.3f", outputs.getNormalized(MotorOutputs.Motor.BL)));
		BR.setText(String.format("%.3f", outputs.getNormalized(MotorOutputs.Motor.BR)));
		BV.setText(String.format("%.3f", outputs.getNormalized(MotorOutputs.Motor.BV)));
	}

	public void setOutputsToMotors(MotorOutputs outputs) {
		physicaloid.close();
		if(physicaloid.open()) {

			JSONFromatter formatter = new JSONFromatter();
			String jsonOutput = formatter.formatMotorOutputs(outputs);

			this.updateMotorDisplay(outputs);

			byte[] buf = jsonOutput.getBytes();

			tvAppend(text, "Write: " + new String(buf), true);
			physicaloid.write(buf, buf.length);

		} else {
			//Error while connecting
			Toast.makeText(MainActivity.this, "Cannot open for Write", Toast.LENGTH_SHORT).show();
		}
	}


	public void onClickWrite(View v) {
		MotorOutputs outs = new MotorOutputs(new double[]{1, 1, 1, 1, 1, 1});
		setOutputsToMotors(outs);
	}


	public void onClickReset(View v) {
		tvClear(text);
		physicaloid.close();
	}

	@Override
	public void onStart(){
		super.onStart();

		}

	@Override
	protected void init(NodeMainExecutor nodeMainExecutor) {
		
		super.init(nodeMainExecutor);
        try {
            java.net.Socket socket = new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
            java.net.InetAddress local_network_address = socket.getLocalAddress();
            socket.close();
            NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());

			String joyTopic = remaps.get(getString(R.string.joystick_topic));

			NameResolver appNameSpace = getMasterNameSpace();
			joyTopic = appNameSpace.resolve(joyTopic).toString();

			virtualJoystickView.setTopicName(joyTopic);

			nodeMainExecutor.execute(virtualJoystickView,
					nodeConfiguration.setNodeName("android/virtual_joystick"));

			System.out.println("Socket works");

        } catch (IOException e) {
        	System.out.println("Socket error: " + e.getMessage());
        }

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
	public void onDestroy() {
		super.onDestroy();
	}
}
