package Util;

import java.util.BitSet;

/**
 * Created by isaiah on 07/03/18.
 */

public class EmbeddedManager {

    private byte[] input; // 25 bytes max
    private BitSet bitset;

    private static final int TOTAL_NUM_BITS = 194;

    public EmbeddedManager(byte[] input) {
        this.input = input;
        this.bitset = BitSet.valueOf(input);

        this.parseBitSet();
    }

    private Message parseBitSet() {
     return new Message();
    }

    public BitSet formatMessage() {
        return new BitSet();
    }

    public class Message {
        public int embeddedStatus;
        public double[] imuData;
        public int temperature;
        public double[] trustFeedback;
        public int[] motorState;
        public int stateOfCharge;
        public double battery;
        public int stateOfPower; // power you can draw before giving out in the next 5 seconds
        public int switchNum;

        public Message() {
            this.imuData = new double[4];
            this.trustFeedback = new double[6];
            this.motorState = new int[6];
        }
    }


}
