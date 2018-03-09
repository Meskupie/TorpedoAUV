package Util;

import java.util.BitSet;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Set;

/**
 * Created by isaiah on 07/03/18.
 */

public class EmbeddedManager {

    private static final int TOTAL_NUM_BITS = 248;

    private byte[] input; // 31 bytes max
    private int byteIndex = 0;

    private byte[] test;

    public EmbeddedManager(byte[] input) {
        test = new byte[]{0xD,0x9,0x3,0x0,0x0,0x0,0x0,0x0,0xF,0x7,0xE,0xA,0x2,0x1,0x8,0x3,0xE,0xF,0x0,0x0,0x9,0xF,0x1,0x2,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0xF,0xF,0xB,0xC,0x1,0xF,0x3,0x3,0x3,0x3,0x3,0x3,0x0,0x0};

        this.input = test;
        //this.parseBitSet();
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

        double[] motorStatus = readNextNBytes(new DataUnit(Type.uint16_t, 16), 6);
        for(int i = 0; i < motorStatus.length; i++) {
            m.motorStatus[i] = (int)motorStatus[i];
        }

        m.smcStatus = (int)readNextBytes(new DataUnit(Type.uint16_t, 16));
        m.swStatFront = (boolean)readNextBytes(new DataUnit(Type.uint8_t, 1));
        m.swStateCenter = (boolean)readNextBytes(new DataUnit(Type.uint8_t, 1));
        m.swStateRear = (boolean)readNextBytes(new DataUnit(Type.uint8_t, 1));

        return m;
    }

    // TODO: implement a byte by bbyte parsing system, remember little endian, swap first 8 bits with second 8 bist for each 16 bits
    public Object readNextBytes(DataUnit data) {
        int numBits = data.numBits;
        int numBytes = numBits / 8;
        byteIndex += numBytes;
        int value = 0;

        if(byteIndex +1 < this.input.length) {
            value = (int)this.input[byteIndex] << 8 + (int)this.input[byteIndex+1];
        }
        return getFormattedData(value, data.dataType);
    }

    public double[] readNextNBytes(DataUnit data, int n ) {
        double[] arr = new double[n];

        for(int i = 0; i < n; i++) {
            int numBitsToRead = data.numBits;
          //  BitSet bitArray = this.bitset.get(bitIndex, bitIndex + numBitsToRead);
          //  bitIndex += numBitsToRead;

            arr[i] = (double)getFormattedData(10, data.dataType);
        }
        return arr;
    }

    public Object getFormattedData(int bits, Type type){
        int bitsToInt = 0;

//        for (int i = 0; i < bits.length(); ++i) {
//            bitsToInt += bits.get(i) ? (1 << i) : 0;
//        }

        switch(type) {
            case uint16_t:
            case uint8_t:
            case ESC_RUN_STATE:
            case SystemRunState:
                return bitsToInt;
            case int16_t:
                return ((double)bitsToInt)/((2<<15)-1) - 0.5;
            default:
                return 0;
        }
    }

    public class DataUnit {
        Type dataType;
        int numBits;

        DataUnit(Type dataType, int numBits) {
            this.dataType = dataType;
            this.numBits = numBits;
        }

    }

    public class Message {

        public int batteryVoltage;
        public int depth;
        public double[] imuData;
        public double[] motorThrust;
        public int battSOC;
        public int battSOP;
        public int ambientTemp;
        public int[] motorStatus;
        public int smcStatus;
        public boolean swStatFront;
        public boolean swStateCenter;
        public boolean swStateRear;

        public Message() {

            this.imuData = new double[4];
            this.motorThrust = new double[6];
            this.motorStatus = new int[6];
        }
    }

    public enum Type {
        uint16_t, int16_t, uint8_t, ESC_RUN_STATE, SystemRunState
    }
}

/* Struct that is being sent as a byte array
typedef struct {
uint16_t battVoltage_mV:16;
uint16_t depth_m : 16;
int16_t imu_x : 16;
int16_t imu_y : 16;
int16_t imu_z : 16;
int16_t imu_w : 16;
uint16_t motorThrust0_mN : 16;
uint16_t motorThrust1_mN : 16;
uint16_t motorThrust2_mN : 16;
uint16_t motorThrust3_mN : 16;
uint16_t motorThrust4_mN : 16;
uint16_t motorThrust5_mN : 16;
uint8_t battSOC :8;
uint8_t battSOP :8;
uint8_t ambientTemperature_C:8;
ESC_RUN_STATE motorStatus0 : 4;
ESC_RUN_STATE motorStatus1 : 4;
ESC_RUN_STATE motorStatus2 : 4;
ESC_RUN_STATE motorStatus3 : 4;
ESC_RUN_STATE motorStatus4 : 4;
ESC_RUN_STATE motorStatus5 : 4;
SystemRunState SMC_Status : 3;
uint8_t swStateFront : 1;
uint8_t swStateCenter : 1;
uint8_t swStateRear : 1;
uint8_t _filler:2;
} HostUpdateStruct_transmit; //length 31 bytes
*/