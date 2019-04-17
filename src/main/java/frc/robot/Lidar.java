package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;
import frc.robot.genericrobot.SuperMOEva;

public class Lidar {
    static int uArtCounter = 0;
    public static void reset(SerialPort Blinky) {
        SmartDashboard.putString("Resetting the port: ", "Start");
        Blinky.reset();
        SmartDashboard.putString("Resetting the port: ", "End");
    }

    public static Logger<String> lidarLogger = new Logger<>();

    public static void getLidar(GenericRobot us, SerialPort Blinky) {
        String lidarString = "", exception = "", readStringBlinky;
        String[] lString = new String[us.numSensors()];
        int[] l = new int[us.numSensors()];

        //reading string from Blinky
        if (uArtCounter == 5) {
            try {
                readStringBlinky = Blinky.readString();
                lidarLogger.printIfChanged("Lidar",readStringBlinky);
                lidarString = readStringBlinky.substring(0, readStringBlinky.indexOf("0-") + 2);
                //System.out.println("Straight from Blinky: " + lidarString + ";");
                SmartDashboard.putString("Straight from Blinky: ", lidarString);
                SmartDashboard.putString("We caught an error on reading the port", "");
            } catch (Exception e) {
                //exception = "exception " + e;
                //System.out.println("booo " + exception);
                //SmartDashboard.putString("We caught an error on reading the port", exception);
            }
            uArtCounter = 0;
        } else {
            uArtCounter++;
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
            } else if (lidarString.indexOf("1-") == lidarString.length() - 2) {
                for (int k = 1; k < us.numSensors() - 1; k++) {
                    try {
                        lString[k] = lidarString.substring(lidarString.indexOf(k + "-") + 2, lidarString.indexOf((k + 1) + "-")).trim();
                    } catch(Exception e) {}
                }
                try {
                    lString[0] = lidarString.substring(lidarString.indexOf(us.numSensors() - 3 + "-") + 2, lidarString.indexOf("1-")).trim();
                } catch(Exception e) {}
                try {
                    lString[2] = lidarString.substring(lidarString.indexOf(us.numSensors() - 1 + "-") + 2, lidarString.indexOf("0-")).trim();
                } catch(Exception e) {}
                try {
                    lString[1] = lidarString.substring(0, lidarString.indexOf(" "));
                } catch(Exception e) {}
            } else {
                for (int k = 1; k < us.numSensors() - 1; k++) {
                    try {
                        lString[k] = lidarString.substring(lidarString.indexOf(k + "-") + 2, lidarString.indexOf((k + 1) + "-")).trim();
                    } catch(Exception e) {}
                }
                try {
                    lString[0] = lidarString.substring(lidarString.indexOf(us.numSensors() - 3 + "-") + 2, lidarString.indexOf("1-")).trim();
                } catch(Exception e) {}
                try {
                    lString[2] = lidarString.substring(0, lidarString.indexOf(" "));
                } catch(Exception e) {}
            }


            //parsings substrings
            for (int i = 0; i < us.numSensors(); i++) {
                try {
                    l[i] = Integer.parseInt(lString[i]);
                    SmartDashboard.putString("Lidar " + i + " parsing error: ", "");
                } catch (Exception e) {
                    exception = "ERROR " + e;
                    //SmartDashboard.putString("Lidar " + i + " parsing error: ", exception);
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


/*package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;
import java.util.regex.*;

public class Lidar {
    public static LidarThread lidarThread;
    public static int numSensors;

    //Init - instantiate new LidarThread
    public static void init(SerialPort Blinky, GenericRobot us) {
        lidarThread = new LidarThread(Blinky);
        lidarThread.start();
        Lidar.numSensors = us.numSensors();
    }

    public static void serialReset(SerialPort Blinky) {
        SmartDashboard.putString("Resetting the port: ", "Start");
        Blinky.reset();
        SmartDashboard.putString("Resetting the port: ", "End");
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
            lString = new String[Lidar.numSensors];
            lidar = new int[Lidar.numSensors];
            while (isAlive) {
                //Reading string from serial port
                try {
                    lidarString = new String(Blinky.readString());
                    ma = p.matcher(lidarString);
                    SmartDashboard.putString("Straight from Blinky: ", lidarString);
                    SmartDashboard.putString("We caught an error on reading the port", "");
                } catch (Exception e) {
                    SmartDashboard.putString("We caught an error on reading the port", e.toString());
                }

                //Does not parse if lidar is maxing out
                if (lidarString.contains("65535") || lidarString.length() == 0) {
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
            if (lidarThread.lidar[j] != 0) {
                SmartDashboard.putNumber("Lidar " + j + ": ", lidarThread.lidar[j]);
                SmartDashboard.putNumber("LidarReadTime", lidarThread.lidarReadTime);
                us.lidar[j] = lidarThread.lidar[j];
                us.lidarReadTime = lidarThread.lidarReadTime;
            }
        }
    }
}*/
