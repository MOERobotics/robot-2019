package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class Lidar {

    public static void init(SerialPort Blinky) {
        SmartDashboard.putString("Resetting the port: ", "Start");
        Blinky.reset();
    }

    public static void getLidar(GenericRobot us, SerialPort Blinky) {

        //initializing vars
        int num = 0;
        String lidarString = "", exception = "";
        String[] lString = new String[us.numSensors()];

        Pattern p = Pattern.compile("-");
        Matcher ma = p.matcher(lidarString);
        int[] l = new int[us.numSensors()];

        //reading string from Blinky
        try {
            lidarString = new String(Blinky.readString());
            System.out.println("Straight from Blinky: " + lidarString + ";");
            SmartDashboard.putString("Straight from Blinky: ", lidarString);
            SmartDashboard.putString("We caught an error on reading the port", "");
        } catch (Exception e) {
            exception = "exception " + e;
            System.out.println("booo " + exception);
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
                SmartDashboard.putString("lString: ", lString[0]);
            } else {
                if (lidarString.indexOf(" ") != -1)
                lString[num] = lidarString.substring(0, lidarString.indexOf(" "));
                //SmartDashboard.putString("lString: ", lString[0]);
            }


            //TODO: guys for loops are a thing please use them this hurts
            //parsings substrings
            try {
                l[0] = Integer.parseInt(lString[0]);
                SmartDashboard.putString("Lidar 0 parsing error: ", "");
            } catch (Exception e) {
                exception = "ERROR " + e;
                SmartDashboard.putString("Lidar 0 parsing error: ", exception);
            }

            /*try {
                l[1] = Integer.parseInt(lString[1]);
            } catch (Exception e) {
                exception = "ERROR " + e;
                SmartDashboard.putString("Lidar 1 parsing error: ", exception);
            }

            try {
                l[2] = Integer.parseInt(lString[2]);
            } catch (Exception e) {
                exception = "ERROR " + e;
                SmartDashboard.putString("Lidar 2 parsing error: ", exception);
            }

            try {
                l[3] = Integer.parseInt(lString[3]);
            } catch (Exception e) {
                exception = "ERROR " + e;
                SmartDashboard.putString("Lidar 3 parsing error: ", exception);
            }

            try {
                l[4] = Integer.parseInt(lString[4]);
            } catch (Exception e) {
                exception = "ERROR " + e;
                SmartDashboard.putString("Lidar 4 parsing error: ", exception);
            }

            try {
                l[5] = Integer.parseInt(lString[5]);
            } catch (Exception e) {
                exception = "ERROR " + e;
                SmartDashboard.putString("Lidar 5 parsing error: ", exception);
            }

            try {
                l[6] = Integer.parseInt(lString[6]);
            } catch (Exception e) {
                exception = "ERROR " + e;
                SmartDashboard.putString("Lidar 6 parsing error: ", exception);
            }

            try {
                l[7] = Integer.parseInt(lString[7]);
            } catch (Exception e) {
                exception = "ERROR " + e;
                SmartDashboard.putString("Lidar 7 parsing error: ", exception);
            }*/

            SmartDashboard.putNumber("Lidar 0: ", l[0]);
            /*SmartDashboard.putNumber("Lidar 1: ", l[1]);
            SmartDashboard.putNumber("Lidar 2: ", l[2]);
            SmartDashboard.putNumber("Lidar 3: ", l[3]);
            SmartDashboard.putNumber("Lidar 4: ", l[4]);
            SmartDashboard.putNumber("Lidar 5: ", l[5]);
            SmartDashboard.putNumber("Lidar 6: ", l[6]);
            SmartDashboard.putNumber("Lidar 7: ", l[7]);*/

            us.lidar[0] = l[0];
            /*us.lidar[1] = l[1];
            us.lidar[2] = l[2];
            us.lidar[3] = l[3];
            us.lidar[4] = l[4];
            us.lidar[5] = l[5];
            us.lidar[6] = l[6];
            us.lidar[7] = l[7];*/

        }
    }
}
