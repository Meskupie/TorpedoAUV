package com.github.rosjava.android_apps.teleop;

import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.os.Handler;
import android.text.method.ScrollingMovementMethod;
import android.util.Log;
import android.view.DragEvent;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import com.felhr.usbserial.UsbSerialDevice;
import com.felhr.usbserial.UsbSerialInterface;
import com.github.rosjava.android_remocons.common_tools.apps.RosAppActivity;
import com.physicaloid.lib.Physicaloid;
import com.physicaloid.lib.usb.driver.uart.ReadLisener;

import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.view.RosImageView;
import org.ros.android.view.VirtualJoystickView;
import org.ros.message.MessageListener;
import org.ros.namespace.NameResolver;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Subscriber;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;
import org.w3c.dom.NodeList;
import org.xbill.DNS.Serial;

import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.util.HashMap;
import java.util.Iterator;

import Communication.ArduinoManager;
import Communication.JSONFromatter;
import Util.MotorOutputs;
import geometry_msgs.Twist;
import me.aflak.arduino.Arduino;
import me.aflak.arduino.ArduinoListener;
import sensor_msgs.Joy;

public class MainActivity extends RosAppActivity {
	private VirtualJoystickView virtualJoystickView;
	private Button backButton;
	private MapTalker mapTalker;

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

		Transform test_start = new Transform(new Vector3(0,0,0),new Quaternion(0,0,0.383,0.924));
		Transform test_delta = new Transform(new Vector3(1,0,0),new Quaternion(0,0,0,1));
		Transform test_finish = test_start.multiply(test_delta);

		Log.d("DEBUG_MSG", "x:"+test_finish.getTranslation().getX()+", y:"+test_finish.getTranslation().getY()+", z:"+test_finish.getTranslation().getZ());
		Log.d("DEBUG_MSG","x:"+test_finish.getRotationAndScale().getX()+", y:"+test_finish.getRotationAndScale().getY()+", z:"+test_finish.getRotationAndScale().getZ()+", w:"+test_finish.getRotationAndScale().getW());
	}




	@Override
	protected void init(NodeMainExecutor nodeMainExecutor) {
		
		super.init(nodeMainExecutor);
        try {
            java.net.Socket socket = new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
            java.net.InetAddress local_network_address = socket.getLocalAddress();
            socket.close();
            NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());

            // MAP TOPIC
			this.mapTalker = new MapTalker(this);
			nodeMainExecutor.execute(this.mapTalker, nodeConfiguration.setNodeName("android/mapTalker"));

            // JOYSTICK TOPIC
			String joyTopic = remaps.get(getString(R.string.joystick_topic));

			NameResolver appNameSpace = getMasterNameSpace();
			joyTopic = appNameSpace.resolve(joyTopic).toString();

			virtualJoystickView.setTopicName(joyTopic);
			nodeMainExecutor.execute(virtualJoystickView, nodeConfiguration.setNodeName("android/virtual_joystick"));







			System.out.println("Socket works");

        } catch (IOException e) {
           System.out.println("Socket error: " + e.getMessage());

        }
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
