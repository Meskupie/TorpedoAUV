package Communication;

import android.content.BroadcastReceiver;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;

/**
 * Created by meskupie on 16/03/18.
 */

public class USBDeviceWrapper {
    public UsbDevice usbDevice;
    public BroadcastReceiver usbReceiver;
    public UsbDeviceConnection usbConnection;
    public int VENDOR_ID = 0x2341;
    public int PRODUCT_ID = 0x8036;
    public String name;

    public USBDeviceWrapper(String device_name,int VEN_ID, int PROD_ID){
        name = device_name;
        VENDOR_ID = VEN_ID;
        PRODUCT_ID = PROD_ID;
    }



}
