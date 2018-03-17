package Communication;

import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.os.AsyncTask;
import android.util.Log;

import com.felhr.usbserial.UsbSerialDevice;

/**
 * Created by Nick Skupien on 3/9/2018.
 */

// Q when sending the motor data
// q when requesting to get new data

public class AsyncArduinoWrite extends AsyncTask<Object, Void, Void>
{

    @Override
    protected void onPreExecute() {
        super.onPreExecute();
    }

    @Override
    protected Void doInBackground(Object... params) {
    //this method will be running on background thread so don't update UI frome here
    //do your long running http tasks here,you dont want to pass argument and u can access the parent class' variable url over here

        if(params.length != 2) {
            System.out.println("Error: there were too few parameters in the async write call");
            return null;
        }

        UsbSerialDevice serial = (UsbSerialDevice) params[0];
        byte[] buf = (byte[]) params[1];

        if(serial != null) {
            serial.write(buf);
        } else {
            Log.d("ARDUINO:", "serial is null");
        }
;       return null;
    }

    @Override
    protected void onPostExecute(Void result) {
        super.onPostExecute(result);
        //this method will be running on UI thread
    }

}