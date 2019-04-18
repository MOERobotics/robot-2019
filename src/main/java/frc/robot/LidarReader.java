package frc.robot;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.genericrobot.*;

import java.util.HashMap;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class LidarReader extends Thread{
    private Map<Integer, Integer> lidars = new HashMap<>(8);
    SerialPort lidarPort;
    GenericRobot robot;

    public LidarReader (SerialPort lidarPort) {
        this.lidarPort = lidarPort;
    }

    private String workingString = " ";
    private Pattern lidarFormat = Pattern.compile("(?<id>\\d+)-(?<value>\\d+)");
    byte newbyte = (byte) 0;

    public void set(SerialPort Blinky, GenericRobot robot) {
        lidarPort = Blinky;
        this.robot = robot;
    }

    @Override
    public void run() {
        while (true) {
             try {
                 newbyte = lidarPort.read(1)[0];
             } catch (Exception e) {
                 System.out.println(e.toString());
             }

            if (newbyte != ' ') {
                workingString += Character.toString(newbyte);
            } else  {
                if  (!workingString.contains("-")) {
                    workingString = "";
                    continue;
                }
                Matcher lidarBlob  = lidarFormat.matcher(workingString);

                if (!lidarBlob.matches()) {
                    workingString = "";
                    continue;
                }

                int id = Integer.parseInt((lidarBlob.group(   "id")));
                int value = Integer.parseInt(lidarBlob.group("value"));

                lidars.put(id,value);
                workingString = "";
                print();
            }
        }
    }

    public void print() {
        robot.lidar[0] = getValue(0);
        //SmartDashboard.putNumber("Lidar 0: ", getValue(0));
    }

    public Integer getValue(int id) {
        if (lidars.containsKey(id)) {
            return lidars.get(id);
        } else {
            return null;
        }
    }
}
