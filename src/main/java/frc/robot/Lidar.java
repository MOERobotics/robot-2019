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

        //initializing vars
        int num = 0;
        String lidarString = "", exception = "";
        String[] lString = new String[us.numSensors];

        Pattern p = Pattern.compile("-");
        Matcher ma = p.matcher(lidarString);
        int[] l = new int[us.numSensors];

        //reading string from Blinky
        try {
            lidarString = new String(Blinky.readString());
            System.out.println(lidarString + ". ");
            SmartDashboard.putString("We caught an error on reading the port", "");
        } catch (Exception e) {
            exception = "exception " + e;
            SmartDashboard.putString("We caught an error on reading the port", exception);
        }

        if (!lidarString.equals("")) {
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
            try {
                l[0] = Integer.parseInt(lString[0]);
            } catch (Exception e) {
                exception = "ERROR " + e;
                SmartDashboard.putString("Lidar 0 parsing error: ", exception);
            }

            try {
                l[1] = Integer.parseInt(lString[1]);
            } catch (Exception e) {
                exception = "ERROR " + e;
                SmartDashboard.putString("Lidar 1 parsing error: ", exception);
            }

            /*try {
                l[2] = Integer.parseInt(lString[1]);
            } catch (Exception e) {
                exception = "ERROR " + e;
                SmartDashboard.putString("Lidar 1 parsing error: ", exception);
            }

            try {
                l[3] = Integer.parseInt(lString[1]);
            } catch (Exception e) {
                exception = "ERROR " + e;
                SmartDashboard.putString("Lidar 1 parsing error: ", exception);
            }

            try {
                l[4] = Integer.parseInt(lString[1]);
            } catch (Exception e) {
                exception = "ERROR " + e;
                SmartDashboard.putString("Lidar 1 parsing error: ", exception);
            }

            try {
                l[5] = Integer.parseInt(lString[1]);
            } catch (Exception e) {
                exception = "ERROR " + e;
                SmartDashboard.putString("Lidar 1 parsing error: ", exception);
            }

            try {
                l[6] = Integer.parseInt(lString[1]);
            } catch (Exception e) {
                exception = "ERROR " + e;
                SmartDashboard.putString("Lidar 1 parsing error: ", exception);
            }

            try {
                l[7] = Integer.parseInt(lString[1]);
            } catch (Exception e) {
                exception = "ERROR " + e;
                SmartDashboard.putString("Lidar 1 parsing error: ", exception);
            }*/

            SmartDashboard.putNumber("Lidar 0: ", l[0]);
            SmartDashboard.putNumber("Lidar 1: ", l[1]);
            /*SmartDashboard.putNumber("Lidar 2: ", l[2]);
            SmartDashboard.putNumber("Lidar 3: ", l[3]);
            SmartDashboard.putNumber("Lidar 4: ", l[4]);
            SmartDashboard.putNumber("Lidar 5: ", l[5]);
            SmartDashboard.putNumber("Lidar 6: ", l[6]);
            SmartDashboard.putNumber("Lidar 7: ", l[7]);*/

            us.lidar[0] = l[0];
            us.lidar[1] = l[1];
            /*us.lidar[2] = l[2];
            us.lidar[3] = l[3];
            us.lidar[4] = l[4];
            us.lidar[5] = l[5];
            us.lidar[6] = l[6];
            us.lidar[7] = l[7];*/

        }
    }
}