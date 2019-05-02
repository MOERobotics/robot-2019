package frc.robot.autonomous.unused;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.vision.LidarReader;
import frc.robot.genericrobot.GenericRobot;
import java.util.regex.*;

public class LidarWithThread {
    public static LidarThread lidarThread;
    public static LidarReader lidarReader;

    public static int numSensors;

    //Init - instantiate new LidarThread
    public static void init(SerialPort Blinky) {
        lidarThread.Blinky = Blinky;
        lidarThread.start();
    }

    public static void serialReset(SerialPort Blinky) {
        SmartDashboard.putString("Resetting the port: ", "Start");
        Blinky.reset();
        SmartDashboard.putString("Resetting the port: ", "End");
    }

    public static void displayLidar(SerialPort Blinky) {
        try {
            SmartDashboard.putString("Straight from Blinky!!!: ", Blinky.readString());
        } catch (Exception e) {}
    }

    public static class LidarThread extends Thread {
        public SerialPort Blinky;
        public String lidarString = "";
        public boolean isAlive = true;
        int lidar[];// = new int[];
        public long lidarReadTime;

        Pattern p = Pattern.compile("-");
        Matcher ma;
        int num;
        String[] lString;// = new String[numSensors];

        public LidarThread(SerialPort Blinky) {
            this.Blinky = Blinky;
        }

        @Override
        public void run() {
            lString = new String[LidarWithThread.numSensors];
            lidar = new int[LidarWithThread.numSensors];
            while (isAlive) {
                //Reading string from serial port
                try {
                    lidarString = new String(Blinky.readString());
                    ma = p.matcher(lidarString);
                    SmartDashboard.putString("We caught an error on reading the port", "");
                } catch (Exception e) {
                    SmartDashboard.putString("We caught an error on reading the port", e.toString());
                }

                //Does not parse if lidar is maxing out
                if (lidarString.contains("65535") || lidarString.length() == 0
                    || !lidarString.contains("-")) {
                    continue;
                }

                //Splitting and parsing string if not maxing out
                //Splitting using Matcher
                while (ma.find()) {
                    String numStr = new String(lidarString.substring(ma.start() - 1, ma.start()).trim());
                    num = Integer.parseInt(numStr);
                    int space = lidarString.indexOf(" ", ma.start());

                    if (space < lidarString.length() - 2 && space > 0)
                        lString[num] = lidarString.substring(ma.start() + 1, space).trim();

                    else if (space == -1)
                        lString[num] = lidarString.substring(0, lidarString.indexOf(" "));
                }

                if (lidarString.indexOf(" ") != -1) {
                    String numStr = Character.toString(lidarString.charAt(lidarString.length() - 2));
                    num = Integer.parseInt(numStr);
                    lString[num] = lidarString.substring(0, lidarString.indexOf(" "));
                }

                //Parsing
                for (int i = 0; i < numSensors; i++) {
                    try {
                        lidar[i] = Integer.parseInt(lString[i]);
                        lidarReadTime = System.currentTimeMillis();
                        SmartDashboard.putString("Lidar " + i + " parsing error: ", "");
                    } catch (Exception e) { }
                }
            }
        }
    }

    public static void getLidar(GenericRobot us) {
        //Sends lidar values, read time to robot
        for (int j = 0; j < us.numSensors(); j++) {
            if (lidarReader.getValue(j) != 0) {
                SmartDashboard.putNumber("Lidar " + j + ": ", lidarReader.getValue(j));
                us.lidar[j] = lidarReader.getValue(j);
            }
        }
    }
}
