package Communication;

import android.os.Environment;
import android.util.Log;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Date;

/**
 * Created by isaiah on 28/02/18.
 */

public class Logger {

    private String fileName;
    public static BufferedWriter out;

    public Logger(String fileName) {
        this.fileName = fileName;

        try {
            this.createFileOnDevice(Boolean.TRUE);
        } catch(IOException e) {
            Log.d("Isaiah", "IOException when logging");
        }

    }

    private void createFileOnDevice(Boolean append) throws IOException {
        /*
         * Function to initially create the log file and it also writes the time of creation to file.
         */
        File Root = Environment.getExternalStorageDirectory();

        if(Root.canWrite()){
            String fileNameAndExt = fileName + ".txt";

         /*   //ensure there is only one of this type of file
            File tmpDir = new File(fileNameAndExt);
            if(tmpDir.exists() && fileNameAndExt.length() >=5 ) {
                String currNum = tmpDir.getName().substring(fileNameAndExt.length() - 5, fileNameAndExt.length() - 5);

            }*/


            File  LogFile = new File(Root, fileNameAndExt);
            FileWriter LogWriter = new FileWriter(LogFile, append);
            out = new BufferedWriter(LogWriter);
            Date date = new Date();
            out.write("Logged at" + String.valueOf(date.getHours() + ":" + date.getMinutes() + ":" + date.getSeconds() + "\n"));
            out.close();

        }
    }

    public void writeToFile(String message) {
        try {
            out.write(message + "\n");
            out.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
