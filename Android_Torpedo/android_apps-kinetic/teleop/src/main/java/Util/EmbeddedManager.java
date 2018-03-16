package Util;

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

    private static final int TOTAL_NUM_BITS = 248;

    private byte[] input; // 31 bytes max
    private int byteIndex = 0;
    private int bitIndex = 0;

    private byte[] test;

    // test = new byte[]{0xD,0x9,0x3,0x0,0x0,0x0,0x0,0x0,0xF,0x7,0xE,0xA,0x2,0x1,0x8,0x3,0xE,0xF,0x0,0x0,0x9,0xF,0x1,0x2,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0xF,0xF,0xB,0xC,0x1,0xF,0x3,0x3,0x3,0x3,0x3,0x3,0x0,0x0};

    public Message parseBytes(byte[] input) {
        this.input = input;
        return this.parseBitSet();
    }


    public Message parseBitSet() {

        Message m = new Message();

        m.batteryVoltage = (int)readNextBytes(new DataUnit(Type.uint16_t, 16));
        m.depth = (int)readNextBytes(new DataUnit(Type.uint16_t, 16));
        m.imuData = readNextNBytes(new DataUnit(Type.uint16_t, 16),4 );
        m.motorThrust = readNextNBytes(new DataUnit(Type.uint16_t, 16),6);
        m.battSOC = (int)readNextBytes(new DataUnit(Type.uint8_t, 8));
        m.battSOP = (int)readNextBytes(new DataUnit(Type.uint8_t, 8));
        m.ambientTemp = (int)readNextBytes(new DataUnit(Type.uint16_t, 16));

        int[] motorStatus = readNextNBytes(new DataUnit(Type.uint16_t, 16), 6);
        for(int i = 0; i < motorStatus.length; i++) {
            m.motorStatus[i] = motorStatus[i];
        }

        m.smcStatus = (int)readNextBytes(new DataUnit(Type.uint16_t, 16));
        m.swStatFront = (int)readNextBytes(new DataUnit(Type.uint8_t, 1)) == 1;
        m.swStateCenter = (int)readNextBytes(new DataUnit(Type.uint8_t, 1)) == 1;
        m.swStateRear = (int)readNextBytes(new DataUnit(Type.uint8_t, 1)) == 1;

        return m;
    }

    private Object readNextBytes(DataUnit data) {
        int numBits = data.numBits;
        int numBytes = numBits / 8;

        int value = 0;

        //assumes that disjointed bits occur at end of parsing, and total less than 1 byte
        if(numBits < 8) {
            value = ((1 << (7 - bitIndex)) & numBits) != 0 ? 1 : 0;
            bitIndex += numBits;
            return getFormattedData(value, data.dataType);
        } else if(numBits == 8) {
            if(byteIndex < this.input.length)
            value = (int)this.input[byteIndex];
            byteIndex += numBytes;
            return getFormattedData(value, data.dataType);
        } else if ( numBits == 16){

            if(byteIndex + 1 < this.input.length) {
                value = ((this.input[byteIndex]) & 0xFF) | (((int)this.input[byteIndex+1]) << 8);
            }

            byteIndex += numBytes;

            return getFormattedData(value, data.dataType);
        } else {
            System.out.println("Error data type not supported");
            return 0;

        }


    }

    private int[] readNextNBytes(DataUnit data, int n ) {
        int[] arr = new int[n];

        for(int i = 0; i < n; i++) {
            int numBitsToRead = data.numBits;
            arr[i] = (int) readNextBytes(data);
        }
        return arr;
    }

    private Object getFormattedData(int bits, Type type){
        int bitsToInt = 0;
        bitsToInt = bits;
        switch(type) {
            case uint16_t:
            case uint8_t:
            case ESC_RUN_STATE:
            case SystemRunState:
                return bitsToInt;
            case int16_t:
                return bitsToInt;//((double)bitsToInt)/((2<<15)-1) - 0.5;
            default:
                return 0;
        }
    }

    public class DataUnit {
        Type dataType;
        int numBits;

        DataUnit(Type dataType, int numBits) {
//            this.dataType = dataType;
//            this.numBits = numBits;
//            byte[] bytes = ByteBuffer.allocate(4).putInt(1695609641).array();
//
//            for (byte b : bytes) {
//                System.out.format("0x%x ", b);
//            }
        }

    }

    public String formatMessageToSend(MotorOutputs outputs, int status) {
        outputs.get(MotorOutputs.Motor.FL).;
    }


    public class Message {

        public int batteryVoltage;
        public int depth;
        public int[] imuData;
        public int[] motorThrust;
        public int battSOC;
        public int battSOP;
        public int ambientTemp;
        public int[] motorStatus;
        public int smcStatus;
        public boolean swStatFront;
        public boolean swStateCenter;
        public boolean swStateRear;

        public Message() {

            this.imuData = new int[4];
            this.motorThrust = new int[6];
            this.motorStatus = new int[6];
        }

        @Override
        public String toString()
        {
            String str = "";
            str += "batteryVoltage" + batteryVoltage + '\n';
            str += "depth" + depth + '\n';
            str += "imuData[0]" + imuData[0] + '\n';
            str += "imuData[1]" + imuData[1] + '\n';
            str += "motorThrust0" + motorThrust[0] + '\n';
            str += "motorThrust1" + motorThrust[1] + '\n';
            str += "battSOC" + battSOC + '\n';
            str += "battSOP" + battSOP + '\n';
            str += "ambientTemp" + ambientTemp + '\n';
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


}
