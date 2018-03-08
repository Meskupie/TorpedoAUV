package Communication;

import android.app.Application;
import android.content.Context;
import android.hardware.usb.UsbDevice;
import android.util.Log;
import android.widget.Toast;

import com.github.rosjava.android_apps.teleop.MainActivity;

import me.aflak.arduino.Arduino;
import me.aflak.arduino.ArduinoListener;

/**
 * Created by isaiah on 28/02/18.
 */

public class ArduinoManager {

    public class MyArduino extends Arduino {

        private Context c;

        MyArduino(Context context) {
            super(context);
            this.c = context;
        }

        Context getContext() {
            return this.c;
        }
    }

    private MyArduino arduino;

    public ArduinoManager(Context c) {

            this.arduino = new MyArduino(c);

            if(this.arduino == null) {
                Log.d("arduino", "THE ARDUINO IS null!!");
            } else {
                Log.d("arduino", this.arduino.toString());
            }

            this.arduino.setArduinoListener(new ArduinoListener() {

            @Override
            public void onArduinoAttached(UsbDevice usbDevice) {
                arduino.open(usbDevice);
                Toast.makeText(arduino.getContext(), "Arduino Attached!!!!", Toast.LENGTH_SHORT).show();

            }

            @Override
            public void onArduinoDetached() {
                Toast.makeText(arduino.getContext(), "Arduino Detached!!!!", Toast.LENGTH_SHORT).show();
            }

            @Override
            public void onArduinoMessage(byte[] bytes) {
                String message = new String(bytes);
               // Logger logger = new Logger("AndroidToArduinoLog");
              //  logger.writeToFile(message);


                Toast.makeText(arduino.getContext(), "HELLO WORLD!!!!", Toast.LENGTH_SHORT).show();
            }

            @Override
            public void onArduinoOpened() {
                String str = "hello world!";
                arduino.send(str.getBytes());
                Toast.makeText(arduino.getContext(), "Arduino Opened!!!!", Toast.LENGTH_SHORT).show();
            }
        });
    }

    public void safeClose() {
        this.arduino.unsetArduinoListener();
        this.arduino.close();
    }

}
