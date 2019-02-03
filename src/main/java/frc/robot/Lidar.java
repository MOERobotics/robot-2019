package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class Lidar {

    public static void init(SerialPort Blinky) {
        SmartDashboard.putString("Resetting the port: ", "Start");
        Blinky.reset();
        SmartDashboard.putString("Resetting the port: ", "Done");
    }

    public static void getLidar(Robot us, SerialPort Blinky) {
        int num = 0;
        String lidarString = "", exception = "";
        String[] lString = new String[us.numSensors];
        Pattern p = Pattern.compile("-");
        Matcher ma = p.matcher(lidarString);
        int[] l = new int[us.numSensors];

        //reading string from Blinky
        try{
            lidarString = new String(Blinky.readString());
            SmartDashboard.putString("straight from blinky: ", lidarString);
            //System.out.println(lidarString + ";");
            SmartDashboard.putString("We caught an error on reading the port", "");
        }
        catch(Exception e){
            exception = "exception " +  e;
            SmartDashboard.putString("We caught an error on reading the port", exception);
        }

        //splitting
        if (ma.find()) {
            String numStr = lidarString.substring(ma.start() - 1, ma.start()).trim();

            num = Integer.parseInt(numStr);
            int space = lidarString.indexOf(" ", ma.start());

            if (space < lidarString.length() - 2 && space > 0)
                lString[num] = lidarString.substring(ma.start() + 1, space).trim();

            else if (space == -1)
                lString[num] = lidarString.substring(0, lidarString.indexOf(" "));
        }

        //parsings substrings
        for (int j = 0; j < us.numSensors; j++) {
            try {
                l[j] = Integer.parseInt(lString[j]);
            }
            catch(Exception e) {
                exception = "ERROR " + e;
                //SmartDashboard.putString("Lidar parsing error: ", exception);
            }
        }

        for (int put = 0; put < us.numSensors; put++) {
            SmartDashboard.putNumber("Lidar " + put + ": ", l[put]);
        }

        for (int set = 0; set < us.numSensors; set++) {
            us.lidar[set] = l[set];
        }
    }
}
