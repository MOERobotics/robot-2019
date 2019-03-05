package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;

public class Lidar {
    static int i = 0;
    public static void init(SerialPort Blinky) {
        SmartDashboard.putString("Resetting the port: ", "Start");
        Blinky.reset();
        SmartDashboard.putString("Resetting the port: ", "End");
    }

    public static void getLidar(GenericRobot us, SerialPort Blinky) {

        //initializing vars
        int num = 0;
        String lidarString = "", exception = "";
        String[] lString = new String[us.numSensors()];

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
            if (lidarString.indexOf(" ") != -1)
                lString[num] = lidarString.substring(0, lidarString.indexOf(" "));

            /*if (lidarString.indexOf("1-") == lidarString.length() - 2) {
                lString[1] = lidarString.substring(0, lidarString.indexOf("0-")).trim();
                lString[0] = lidarString.substring(lidarString.indexOf("0-") + 2, lidarString.lastIndexOf(" ")).trim();
            } else if (lidarString.indexOf("0-") == lidarString.length() - 2) {
                lString[0] = lidarString.substring(0, lidarString.indexOf("1-")).trim();
                lString[1] = lidarString.substring(lidarString.indexOf("1-") + 2, lidarString.lastIndexOf(" ")).trim();
            } */

            //parsings substrings
            for (i = 0; i < us.numSensors(); i++) {
                try {
                    l[i] = Integer.parseInt(lString[0]);
                    SmartDashboard.putString("Lidar " + i + " parsing error: ", "");
                } catch (Exception e) {
                    exception = "ERROR " + e;
                    SmartDashboard.putString("Lidar " + i + " parsing error: ", exception);
                }
            }

            SmartDashboard.putNumber("Lidar 0: ", l[0]);
            //SmartDashboard.putNumber("Lidar 1: ", l[1]);
            /*SmartDashboard.putNumber("Lidar 2: ", l[2]);
            SmartDashboard.putNumber("Lidar 3: ", l[3]);
            SmartDashboard.putNumber("Lidar 4: ", l[4]);
            SmartDashboard.putNumber("Lidar 5: ", l[5]);
            SmartDashboard.putNumber("Lidar 6: ", l[6]);
            SmartDashboard.putNumber("Lidar 7: ", l[7]);*/

            us.lidar[0] = l[0];
            //us.lidar[1] = l[1];
            /*us.lidar[2] = l[2];
            us.lidar[3] = l[3];
            us.lidar[4] = l[4];
            us.lidar[5] = l[5];
            us.lidar[6] = l[6];
            us.lidar[7] = l[7];*/

        }
    }
}
