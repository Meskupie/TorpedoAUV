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
<<<<<<< Updated upstream
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;
=======
import org.ros.node.parameter.ParameterTree;
>>>>>>> Stashed changes

import java.io.IOException;

public class MainActivity extends RosAppActivity {
	private VirtualJoystickView virtualJoystickView;
	private Button backButton;
<<<<<<< Updated upstream
	private testing test_code;
=======
	private ControllerNode controller_node;
	private ParameterNode parameter_node;
>>>>>>> Stashed changes

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
        parameter_node = new ParameterNode();
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

		parameter_node.setDynamics("data.txt");//"rough_controller_data.txt");

//		std::fstream controller_file((CONTROLLER_FILE_LOCATION+controller_filename+".txt").c_str());
//		if(controller_file.is_open()){
//			float temp;
//			for(int i = 0; i < 144; i++){
//				controller_file >> temp;
//				system_parameters.model_A.push_back(temp);
//			}
//			for(int i = 0; i < 72; i++){
//				controller_file >> temp;
//				system_parameters.model_B.push_back(temp);
//			}
//			for(int i = 0; i < 72; i++){
//				controller_file >> temp;
//				system_parameters.lqr_K.push_back(temp);
//			}
//			controller_file.close();
//		}
//		else{
//			ROS_ERROR("Controller file failed to load");
//		}

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
