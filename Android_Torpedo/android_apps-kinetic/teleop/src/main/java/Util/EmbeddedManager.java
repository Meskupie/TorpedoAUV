package Util;

import android.util.Log;

import org.apache.commons.lang.builder.ToStringBuilder;

import java.util.BitSet;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Set;

/**
 * Created by isaiah on 07/03/18.
 */

// q Read, Q Write
// upper case set, lower case read

public class EmbeddedManager {

    private static final String TAG = "ROV_DEBUG";

    private byte[] input; // 33 bytes max
    private int byteIndex = 0;
    private int bitIndex = 0;

    public static final String SMC_WRITE = "X";
    public static final String SMC_READ = "x";

    private byte[] test;

    public SMCSensors parseBytes(byte[] input) {
        this.input = input;
        return this.parseBitSet();
    }

    public SMCSensors parseBitSet() {

        SMCSensors m = new SMCSensors();

        m.batteryVoltage = (int)readNextBytes(new DataUnit(Type.uint16_t, 16));
        m.depth = (int)readNextBytes(new DataUnit(Type.uint16_t, 16));
        m.imuData = readNextNBytes(new DataUnit(Type.int16_t, 16),4 );
        m.motorThrust = readNextNBytes(new DataUnit(Type.int16_t, 16),6);
        m.battSOC = (int)readNextBytes(new DataUnit(Type.uint8_t, 8));
        m.battSOP = (int)readNextBytes(new DataUnit(Type.uint8_t, 8));
        m.battCurrent_mA = (int)readNextBytes(new DataUnit(Type.int16_t, 16));
        m.ambientTemp = (int)readNextBytes(new DataUnit(Type.uint16_t, 16));

        int[] motorStatus = readNextNBytes(new DataUnit(Type.uint16_t, 4), 6);
        for(int i = 0; i < motorStatus.length; i++) {
            m.motorStatus[i] = motorStatus[i];
        }

        m.smcStatus = (int)readNextBytes(new DataUnit(Type.uint16_t, 16));
        m.swStatFront = (int)readNextBytes(new DataUnit(Type.uint8_t, 1)) == 1;
        m.swStateCenter = (int)readNextBytes(new DataUnit(Type.uint8_t, 1)) == 1;
        m.swStateRear = (int)readNextBytes(new DataUnit(Type.uint8_t, 1)) == 1;

        return m;
    }


    // TODO: fix ambient temp, add check for 4 bits and 3 bit status
    private Object readNextBytes(DataUnit data) {
        int numBits = data.numBits;
        int numBytes = numBits / 8;

        int value = 0;

        //assumes that disjointed bits occur at end of parsing, and total less than 1 byte
        if(numBits < 8) {
            value = ((1 << (7 - bitIndex)) & numBits) != 0 ? 1 : 0;
            bitIndex += numBits;
        } else if(numBits == 8) {
            Log.d(TAG, "weird things");
            if(byteIndex < this.input.length)
            value = ((int)this.input[byteIndex]) & 0xFF;

            byteIndex += numBytes;
        } else if ( numBits == 16){
           Log.d(TAG, "byteNum: " + byteIndex);

            if(byteIndex + 1 < this.input.length) {
                // check if the number is negative, if so, then twos complement it
                value = (short)((this.input[byteIndex]) & 0xFF) | (((int)this.input[byteIndex+1] & 0xFF) << 8);

                if((this.input[byteIndex+1]  & (1 << 7)) != 0) { // first binary digit was 1
                    value = ~(value-1) & 0xFFFF;
                    value *= -1;
                }

                Log.d(TAG, "shifted A: " + ((this.input[byteIndex]) & 0xFF));
                Log.d(TAG, "shefted B: " + (((int)this.input[byteIndex+1]) << 8));
                Log.d(TAG, "value: " + value);
            }


            byteIndex += numBytes;
        } else {
            System.out.println("Error data type not supported");
            return 0;
        }
        return value;
    }

    private int[] readNextNBytes(DataUnit data, int n ) {
        int[] arr = new int[n];
        for(int i = 0; i < n; i++) {
            int numBitsToRead = data.numBits;
            arr[i] = (int) readNextBytes(data);
        }
        return arr;
    }

    public class DataUnit {
        Type dataType;
        int numBits;

        DataUnit(Type dataType, int numBits) {
            this.dataType = dataType;
            this.numBits = numBits;
        }

    }

    public class SMCSensors {

        public int batteryVoltage;
        public int depth;
        public int[] imuData;
        public int[] motorThrust;
        public int battSOC;
        public int battSOP;
        public int battCurrent_mA;
        public int ambientTemp;
        public int[] motorStatus;
        public int smcStatus;
        public boolean swStatFront;
        public boolean swStateCenter;
        public boolean swStateRear;


        public SMCSensors() {

            this.imuData = new int[4];
            this.motorThrust = new int[6];
            this.motorStatus = new int[6];
        }

        @Override
        public String toString()
        {
            String str = "";
            str += "batteryVoltage: " + batteryVoltage + '\n';
            str += "depth: " + depth + '\n';
            str += "imuData[0]: " + imuData[0] + '\n';
            str += "imuData[1]: " + imuData[1] + '\n';
            str += "imuData[2]: " + imuData[2] + '\n';
            str += "imuData[3]: " + imuData[3] + '\n';
            str += "motorThrust0: " + motorThrust[0] + '\n';
            str += "motorThrust1: " + motorThrust[1] + '\n';
            str += "motorThrust2: " + motorThrust[2] + '\n';
            str += "motorThrust3: " + motorThrust[3] + '\n';
            str += "motorThrust4: " + motorThrust[4] + '\n';
            str += "motorThrust5: " + motorThrust[5] + '\n';
            str += "battSOC: " + battSOC + '\n';
            str += "battSOP: " + battSOP + '\n';
            str += "battCurrent_mA:" + battCurrent_mA + '\n';
            str += "ambientTemp: " + ambientTemp + '\n';
            str += "motorStatus[i]: " + motorStatus[0];

            // add more if necessary

            return str;
        }
    }

    public enum Type {
        uint16_t, int16_t, uint8_t, ESC_RUN_STATE, SystemRunState
    }

    public byte[] hexStringToByteArray(String s) {
        int len = s.length();
        byte[] data = new byte[len / 2];
        for (int i = 0; i < len; i += 2) {
            data[i / 2] = (byte) ((Character.digit(s.charAt(i), 16) << 4)
                    + Character.digit(s.charAt(i+1), 16));
        }
        return data;
    }

    public static byte[] getEmbeddedByteArray(int[] motorOutputs, int numberOfMessagesSent) {
        return new byte[]{(byte)0xFF,(byte)0xFF,(byte)0xFF,(byte)0xFF,(byte)0xFF,(byte)0xFF,(byte)0xFF,(byte)0xFF,(byte)0xFF,(byte)0xFF,(byte)0xFF,(byte)0xFF,(byte)0xFF};
        /*StringBuilder ab = new StringBuilder();
        ab.append(WRITE_COMMAND);

        int checkSum = 0;
        byte[] bytes = new byte[13];

        for(int i = 0; i < 6; i++) {
            bytes[i] = (byte)((motorOutputs[i] & 0xFF00) >> 8);
            bytes[i+1] = (byte)(motorOutputs[i] & 0xFF);
            checkSum += bytes[i] + bytes[i+1];
        }

        checkSum += numberOfMessagesSent;
        checkSum %= 16;

        bytes[12] = (byte)(((numberOfMessagesSent & 0xF) << 4) | (checkSum & 0xF));
        return bytes;*/
    }
}
