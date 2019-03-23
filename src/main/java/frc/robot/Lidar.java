package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;
import frc.robot.genericrobot.SuperMOEva;

public class Lidar {

    public static void reset(SerialPort Blinky) {
        SmartDashboard.putString("Resetting the port: ", "Start");
        Blinky.reset();
        SmartDashboard.putString("Resetting the port: ", "End");
    }

    public static void getLidar(GenericRobot us, SerialPort Blinky) {
        String lidarString = "", exception = "";
        String[] lString = new String[us.numSensors()];
        int[] l = new int[us.numSensors()];

        //reading string from Blinky
        try {
            lidarString = new String(Blinky.readString());
            //System.out.println("Straight from Blinky: " + lidarString + ";");
            SmartDashboard.putString("Straight from Blinky: ", lidarString);
            if (lidarString.equals("")) {
                //System.out.println("getting nothing");
            }
            SmartDashboard.putString("We caught an error on reading the port", "");
        } catch (Exception e) {
            exception = "exception " + e;
            System.out.println("booo " + exception);
            SmartDashboard.putString("We caught an error on reading the port", exception);
        }

        if (!lidarString.equals("")) {

            //splitting into substrings
            if (lidarString.indexOf("0-") == lidarString.length() - 2) {
                for (int k = 1; k < us.numSensors() - 1; k++) {
                    try {
                        lString[k] = lidarString.substring(lidarString.indexOf(k + "-") + 2, lidarString.indexOf((k + 1) + "-")).trim();
                    } catch(Exception e) {}
                }
                try {
                    lString[us.numSensors()-1] = lidarString.substring(lidarString.indexOf(us.numSensors() - 1 + "-") + 2, lidarString.indexOf("0-")).trim();
                } catch(Exception e) {}
                try {
                    lString[0] = lidarString.substring(0, lidarString.indexOf(" "));
                } catch(Exception e) {}
            } else {
                for (int k = 1; k < us.numSensors() - 1; k++) {
                    try {
                        lString[k] = lidarString.substring(lidarString.indexOf(k + "-") + 2, lidarString.indexOf((k + 1) + "-")).trim();
                    } catch(Exception e) {}
                }
                try {
                    lString[0] = lidarString.substring(lidarString.indexOf(us.numSensors() - 2 + "-") + 2, lidarString.indexOf("1-")).trim();
                } catch(Exception e) {}
                try {
                    lString[1] = lidarString.substring(0, lidarString.indexOf(" "));
                } catch(Exception e) {}
            }


            //parsings substrings
            for (int i = 0; i < us.numSensors(); i++) {
                try {
                    l[i] = Integer.parseInt(lString[i]);
                    SmartDashboard.putString("Lidar " + i + " parsing error: ", "");
                } catch (Exception e) {
                    exception = "ERROR " + e;
                    SmartDashboard.putString("Lidar " + i + " parsing error: ", exception);
                    //System.out.println("lString[1] is " + lString[1]);
                }
            }

            for (int j = 0; j < us.numSensors(); j++) {
                if (l[j] > 0) {
                    SmartDashboard.putNumber("Lidar " + j + ": ", l[j]);
                    us.lidar[j] = l[j];
                }
            }

        }
    }
}
