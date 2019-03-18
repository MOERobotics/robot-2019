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
            //System.out.println("Straight from Blinky: " + lidarString + ";");
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
                try {
                    lString[1] = lidarString.substring(0, lidarString.indexOf("0-")).trim();
                } catch (Exception e) {
                    System.out.println(e);
                }
                try {
                    lString[0] = lidarString.substring(lidarString.indexOf("0-") + 2, lidarString.lastIndexOf(" ")).trim();
                } catch (Exception e) {
                    System.out.println(e);
                }
            } else if (lidarString.indexOf("0-") == lidarString.length() - 2) {
                try {
                    lString[0] = lidarString.substring(0, lidarString.indexOf("1-")).trim();
                } catch (Exception e) {
                    System.out.println(e);
                }
                try {
                    lString[1] = lidarString.substring(lidarString.indexOf("1-") + 2, lidarString.lastIndexOf(" ")).trim();
                } catch (Exception e) {
                    System.out.println(e);
                }
            }*/

            //parsings substrings
            for (i = 0; i < us.numSensors(); i++) {
                try {
                    l[i] = Integer.parseInt(lString[i]);
                    SmartDashboard.putString("Lidar " + i + " parsing error: ", "");
                } catch (Exception e) {
                    exception = "ERROR " + e;
                    SmartDashboard.putString("Lidar " + i + " parsing error: ", exception);
                }
            }

            for (int j = 0; j < us.numSensors(); j++) {
                if (l[j] != 0) {
                    SmartDashboard.putNumber("Lidar " + j + ": ", l[j]);
                    us.lidar[j] = l[j];
                }
            }

        }
    }
}
