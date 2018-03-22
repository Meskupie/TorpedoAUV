package Communication;

import android.util.Log;
import org.ros.rosjava_geometry.Quaternion;

/**
 * Created by Michael on 17/03/18.
 * Built off of "EmbeddedManager" by Isaiah.
 */

public class MessageManager {
    private static final String TAG = "ROV_DEBUG";
    private static final String TAG_ERROR = "ROV_ERROR";

    // public messages
    public MsgSMCSensors msg_smc_sensors;
    public MsgSMCMotors msg_smc_motors;
    public MsgCameraTargets msg_front_targets;
    public MsgCameraTargets msg_rear_targets;
    public MsgCameraType msg_camera_type;

    private int checksum_counter = 0;

    public MessageManager(){
        msg_smc_sensors = new MsgSMCSensors();
        msg_smc_motors = new MsgSMCMotors();
        msg_front_targets = new MsgCameraTargets();
        msg_rear_targets = new MsgCameraTargets();
    }

    private int[] parseRawBytes(byte[] raw,int[] bit_size,int[] arr_size,int output_size){
        int byte_index = 0;
        int bit_index = 0;
        int output_index = 0;
        int[] output = new int[output_size];
        // NOTE: THIS DOES NOT HANDLE BIT LEVEL TRANSITIONS WELL, BE CAREFUL
        for(int i = 0; i < bit_size.length; i++){
            for(int j = 0; j < arr_size[i]; j++){ // If there are multiple values
                if(output_index >= output_size){
                    Log.d(TAG_ERROR, "Parser overran output array");
                }else{
                    if (bit_size[i] == 16) {
                        if (bit_index % 8 != 0) {
                            Log.d(TAG_ERROR, "Parser cant handle partial byte case (16 overrun)");
                        }
                        if (bit_index + bit_size[i] <= raw.length * 8) {
                            int assembled = (short) ((raw[byte_index]) & 0xFF) | (((short) raw[byte_index + 1] & 0xFF) << 8);
                            // fix_sign
                            if ((assembled & 32768) != 0) { // first binary digit was 1
                                assembled = ~(assembled - 1) & 0xFFFF;
                                assembled *= -1;
                            }
                            output[output_index] = assembled;
                            output_index++;
                            bit_index += bit_size[i];
                            byte_index = bit_index / 8;
                        } else {
                            Log.d(TAG_ERROR, "Parser overrun 16");
                        }
                    } else if (bit_size[i] == 8) {
                        if (bit_index % 8 != 0) {
                            Log.d(TAG_ERROR, "Parser cant handle partial byte case (8 overrun)");
                        }
                        if (bit_index + bit_size[i] <= raw.length * 8) {
                            int assembled = ((raw[byte_index]) & 0xFF);
                            // fix_sign
                            if ((assembled & 128) != 0) { // first binary digit was 1
                                assembled = ~(assembled - 1) & 0xFF;
                                assembled *= -1;
                            }
                            output[output_index] = assembled;
                            output_index++;
                            bit_index += bit_size[i];
                            byte_index = bit_index / 8;
                        } else {
                            Log.d(TAG_ERROR, "Parser overrun 8");
                        }
                    } else if (bit_size[i] < 8) {
                        if (bit_index + bit_size[i] <= raw.length * 8) {
// This is the fix for that bit stuff order
                            if( ((bit_index % 8) + bit_size[i]) <= 8){
                                int assembled = (raw[byte_index] >> (bit_index % 8)) & ((1 << bit_size[i]) - 1);
                                output[output_index] = assembled;
                                output_index++;
                                bit_index += bit_size[i];
                                byte_index = bit_index / 8;
// This code assumed opposite bit stuff order
//                            int shift = (8 - ((bit_index % 8) + bit_size[i]));
//                            if (shift >= 0) {
//                                int assembled = (raw[byte_index] >> shift) & ((1 << bit_size[i]) - 1);
//                                output[output_index] = assembled;
//                                output_index++;
//                                bit_index += bit_size[i];
//                                byte_index = bit_index / 8;
                            } else {
                                Log.d(TAG_ERROR, "Parser cant handle partial byte case (<8)");
                            }
                        } else {
                            Log.d(TAG_ERROR, "Parser overrun < 8");
                        }
                    } else {
                        Log.d(TAG_ERROR, "Parser cant handle this number of bits");
                    }
                }
            }
        }
        return output;
    }

    private byte[] buildRawBytes(int[] input,int[] bit_size,int[] arr_size, int output_size, boolean include_checksum){
        // HAS RESTRICTIONS ON BIT COUNT
        byte[] output = new byte[output_size+1];
        int byte_index = 1;
        int bit_index = 8;
        int input_index = 0;
        int checksum = 0;
        for(int i = 0; i < bit_size.length; i++){
            for(int j = 0; j < arr_size[i]; j++){
                if(input_index >= input.length){
                    Log.d(TAG_ERROR, "Builder overran input array");
                }else{
                    if (bit_size[i] == 16) {
                        if (bit_index % 8 != 0) {
                            Log.d(TAG_ERROR, "Builder cant handle partial byte case (16)");
                        }
                        if (bit_index + bit_size[i] <= (output_size+1) * 8) {
                            output[byte_index] = (byte)(input[input_index] & 0xFF);
                            output[byte_index+1] = (byte)((input[input_index] & 0xFF00) >> 8);
                            if(include_checksum){
                                checksum += (output[byte_index]+output[byte_index+1]);
                            }
                            input_index ++;
                            bit_index += bit_size[i];
                            byte_index = bit_index / 8;
                        } else {
                            Log.d(TAG_ERROR, "Builder overrun (16)");
                        }
                    } else {
                        Log.d(TAG_ERROR, "Builder cant handle "+bit_size[i]+" bits");
                    }
                }
            }
        }
        // tack on a checksum
        if(include_checksum){
            checksum_counter++;
            checksum += checksum_counter;
            if((bit_index%8 == 0)&&(byte_index == output_size)){
                output[byte_index] = (byte)(((checksum_counter&0xF)<<4)|(checksum&0xF));
            }else{
                Log.d(TAG_ERROR, "Builder ended with an incorrect number of bytes");
            }
        }
        return output;
    }

    public class MsgSMCSensors {
        // Parsing properties
        public final int size_bytes = 33;
        private final int size_values = 26;
        private int[] parsed;
        private int[] bit_size = new int[]{16,16,16,16, 8, 8,16, 8, 4, 3, 1, 1, 1};
        private int[] arr_size = new int[]{ 1, 1, 4, 6, 1, 1, 1, 1, 6, 1, 1, 1, 1};
        private double[] scale = new double[]{0.001,0.001,1.0/32767.0,0.001,1,1,0.001,1,1,1,1,1,1};
        public final byte[] serial_request_tag = new byte[]{0x71}; //q
        //public final byte[] serial_request_tag = new byte[]{0x74}; //t for testing fixed response

        // Data
        public double battery_voltage; // V
        public double depth; // m
        public Quaternion imu; // Quaternion
        public double[] motor_thrust; // N
        public int battery_SOC; // %
        public int battery_SOP; // %
        public double battery_current; // A
        public int temperature; // C
        public int[] motor_status; // enum
        public int smc_status; // enum
        public boolean switch_front; // enum
        public boolean switch_center; // enum
        public boolean switch_rear; // enum

        public MsgSMCSensors() {}

        public void parseData(byte[] raw){
            parsed = parseRawBytes(raw,bit_size,arr_size,size_values);

            battery_voltage = ((double)parsed[0])*scale[0]; // V
            depth = ((double)parsed[1])*scale[1]; // m
            imu = new Quaternion(((double)parsed[2])*scale[2],((double)parsed[3])*scale[2],((double)parsed[4])*scale[2],((double)parsed[5])*scale[2]); // Quaternion
            motor_thrust = new double[]{((double)parsed[10])*scale[3],((double)parsed[6])*scale[3],((double)parsed[11])*scale[3],((double)parsed[7])*scale[3],((double)parsed[8])*scale[3],((double)parsed[9])*scale[3]}; // N
            battery_SOC = parsed[12]*(int)scale[4]; // %
            battery_SOP = parsed[13]*(int)scale[5]; // %
            battery_current = ((double)parsed[14])*scale[6]; // A
            temperature = parsed[15]*(int)scale[7]; // C
            motor_status = new int[]{parsed[16]*(int)scale[8],parsed[17]*(int)scale[8],parsed[18]*(int)scale[8],parsed[19]*(int)scale[8],parsed[20]*(int)scale[8],parsed[21]*(int)scale[8]}; // enum
            smc_status = parsed[22]*(int)scale[9]; // enum
            switch_front = parsed[23]*(int)scale[10]==1; // bool
            switch_center = parsed[24]*(int)scale[11]==1; // bool
            switch_rear = parsed[25]*(int)scale[12]==1; // bool
        }

        public byte[] getRequest(){
            return serial_request_tag;
        }
    }

    public class MsgSMCMotors {
        // Parsing properties
        public final int size_bytes = 13;
        private final int size_values = 6;
        private byte[] raw = new byte[size_bytes+1];
        private int[] bit_size = new int[]{16};
        private int[] arr_size = new int[]{6};
        private double[] scale = new double[]{0.001};
        private final byte[] serial_request_tag = new byte[]{0x6D}; // m
        private final byte[] serial_write_tag = new byte[]{0x4D}; // M

        // Data
        public double[] input_thrust = new double[6]; // N

        public MsgSMCMotors() {}

        public byte[] getBuiltMsg(){
            int[] input = new int[input_thrust.length];
            input[4] = (int)(input_thrust[0]/scale[0]);
            input[0] = (int)(input_thrust[1]/scale[0]);
            input[5] = (int)(input_thrust[2]/scale[0]);
            input[1] = (int)(input_thrust[3]/scale[0]);
            input[2] = (int)(input_thrust[4]/scale[0]);
            input[3] = (int)(input_thrust[5]/scale[0]);
            raw = buildRawBytes(input,bit_size,arr_size,size_bytes,true);
            raw[0] = serial_write_tag[0];
            return raw;
        }

        public byte[] getRequest(){
            return serial_request_tag;
        }
    }

    public class MsgCameraTargets{
        // Parsing properties
        public final int size_bytes = 31;
        private final int size_values = 31;
        private byte[] raw;
        private int[] parsed = new int[size_values];
        private int[] bit_size = new int[]{8,8,8,4,4,8,8,4,4,8,8,4,4,8,8,4,4,8,8,4,4,8,8,4,4,8,8,4,4,8,8,4,4,8,8,4,4,8,8,4,4};
        private int[] arr_size = new int[]{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
        private double[] scale = new double[]{180.0/3.14159,180.0/3.14159,1,0.1,180.0/3.14159,180.0/3.14159,1,0.1,180.0/3.14159,180.0/3.14159,1,0.1,180.0/3.14159,180.0/3.14159,1,0.1,180.0/3.14159,180.0/3.14159,1,0.1,180.0/3.14159,180.0/3.14159,1,0.1,180.0/3.14159,180.0/3.14159,1,0.1,180.0/3.14159,180.0/3.14159,1,0.1,180.0/3.14159,180.0/3.14159,1,0.1,180.0/3.14159,180.0/3.14159,1,0.1};
        private final byte[] serial_request_tag = new byte[]{0x63}; //c

        // Data
        public double[] azumuths = new double [10];
        public double[] altitudes = new double[10];
        public int[] ids = new int[10];
        public double[] confidences = new double[10];

        public MsgCameraTargets(){}

        public void parseData(byte[] raw){
            parsed = parseRawBytes(raw,bit_size,arr_size,size_values);
            for(int i = 0; i < azumuths.length; i++){
                azumuths   [i] = parsed[i*4+0]*scale[i*4+0];
                altitudes  [i] = parsed[i*4+1]*scale[i*4+1];
                ids  [i] = (int)(parsed[i*4+2]*scale[i*4+2]);
                confidences[i] = parsed[i*4+3]*scale[i*4+3];
            }
        }

        public byte[] getRequest(){
            return serial_request_tag;
        }
    }

    public class MsgCameraType{
        // Parsing properties
        private final byte[] serial_request_tag = new byte[]{0x74}; //t

        public byte[] getRequest(){
            return serial_request_tag;
        }
    }
}
