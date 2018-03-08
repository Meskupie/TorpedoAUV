package Communication;

import android.util.Log;

import org.json.JSONException;
import org.json.JSONObject;

import Util.MotorOutputs;

/**
 * Created by isaiah on 01/03/18.
 */

public class JSONFromatter {


    // Example: <{"motorValues":{"FL":0.1,"FR":0.1,"FV":0.1,"BL":0.1,"BR":0.1,"BV":0.1}}>
    private boolean errorInFormatting = false;
    private static final String frontDelimiter = "<";
    private static final String endDelimiter = ">";

    public String formatMotorOutputs(MotorOutputs motorOutputs) {
        JSONObject j = new JSONObject();
        String jsonString = "";

        try {
            JSONObject motorValues = new JSONObject();

            motorValues.put("FL", motorOutputs.get(MotorOutputs.Motor.FL));
            motorValues.put("FR", motorOutputs.get(MotorOutputs.Motor.FR));
            motorValues.put("FV", motorOutputs.get(MotorOutputs.Motor.FV));
            motorValues.put("BL", motorOutputs.get(MotorOutputs.Motor.BL));
            motorValues.put("BR", motorOutputs.get(MotorOutputs.Motor.BR));
            motorValues.put("BV", motorOutputs.get(MotorOutputs.Motor.BV));

            j.put("motorValues", motorValues);
            jsonString =  j.toString();

            this.errorInFormatting = false;
        } catch (JSONException e) {
            Log.d("ERROR", "there was a json formatting error: " + e.getMessage());
            this.errorInFormatting = true;
        }

        return this.setDelimitingChars(jsonString);
   }

   private String setDelimitingChars(String output) {
        return frontDelimiter + output + endDelimiter;
   }

    public boolean hasError() {
        return this.errorInFormatting;
    }


}
