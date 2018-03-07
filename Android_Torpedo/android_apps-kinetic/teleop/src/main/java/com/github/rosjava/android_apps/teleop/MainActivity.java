package com.github.rosjava.android_apps.teleop;

import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;

import com.github.rosjava.android_remocons.common_tools.apps.RosAppActivity;

import org.ros.android.view.VirtualJoystickView;
import org.ros.namespace.NameResolver;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.io.IOException;

public class MainActivity extends RosAppActivity {
	private VirtualJoystickView virtualJoystickView;
	private Button backButton;
	private ControllerNode controller_node;
	private ParametersNode parameter_node;


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


        virtualJoystickView = (VirtualJoystickView) findViewById(R.id.virtual_joystick);
        backButton = (Button) findViewById(R.id.back_button);
        backButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                onBackPressed();
            }
        });
        controller_node = new ControllerNode();
        parameter_node = new ParametersNode();
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
            NodeConfiguration nodeConfiguration =
                    NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());

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

		// Start Controller Node
		try {
			java.net.Socket socket = new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
			java.net.InetAddress local_network_address = socket.getLocalAddress();
			socket.close();
			NodeConfiguration nodeConfiguration =
					NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
			nodeMainExecutor.execute(controller_node,
					nodeConfiguration.setNodeName("android/controller_node"));
		} catch (IOException e) {System.out.println("Socket error: " + e.getMessage());}

		// Start Parameters Node
		try {
			java.net.Socket socket = new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
			java.net.InetAddress local_network_address = socket.getLocalAddress();
			socket.close();
			NodeConfiguration nodeConfiguration =
					NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
			nodeMainExecutor.execute(parameter_node,
					nodeConfiguration.setNodeName("android/parameter_node"));
		} catch (IOException e) {System.out.println("Socket error: " + e.getMessage());}

		// Set parameters
		parameter_node.setDynamics("rough_controller_data.txt");

	}
	
	  @Override
	  public boolean onCreateOptionsMenu(Menu menu){
		  menu.add(0,0,0,R.string.stop_app);

		  return super.onCreateOptionsMenu(menu);
	  }
	  
	  @Override
	  public boolean onOptionsItemSelected(MenuItem item){
		  super.onOptionsItemSelected(item);
		  switch (item.getItemId()){
		  case 0:
			  onDestroy();
			  break;
		  }
		  return true;
	  }
}
