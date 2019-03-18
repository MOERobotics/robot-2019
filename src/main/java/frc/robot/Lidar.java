package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;

public class Lidar {
    GenericRobot us;
    SerialPort Blinky;
    String lidarString = "", exception = "";
    String[] lString = new String[us.numSensors()];

    int[] l = new int[us.numSensors()];

    public void reset() {
        SmartDashboard.putString("Resetting the port: ", "Start");
        Blinky.reset();
        SmartDashboard.putString("Resetting the port: ", "End");
    }

    public void getLidar() {
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
            for (int k = 1; k < us.numSensors() - 1; k++) {
                lString[k] = lidarString.substring(lidarString.indexOf(k+"-") + 2, lidarString.indexOf((k+1)+"-")).trim();
            }
            lString[us.numSensors()-1] = lidarString.substring(lidarString.indexOf(us.numSensors() - 1 + "-") + 2, lidarString.indexOf("0-")).trim();
            lString[0] = lidarString.substring(0, lidarString.indexOf(" "));

            //parsings substrings
            for (int i = 0; i < us.numSensors(); i++) {
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
