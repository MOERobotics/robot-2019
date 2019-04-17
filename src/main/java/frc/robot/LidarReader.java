package frc.robot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public void setLidarPort(SerialPort Blinky, GenericRobot robot) {
        lidarPort = Blinky;
        this.robot = robot;
    }

    @Override
    public void run() {
        while (true) {
            //SmartDashboard.putString("Straight from Blinky", lidarPort.readString());
             try {
                 newbyte = lidarPort.read(1)[0];
                 //System.out.println("LINE 35");
                 //SmartDashboard.putString("Straight from Blinky", lidarPort.read(1).toString());
             } catch (Exception e) {
                 System.out.println(e.toString());
             }

            if (newbyte != ' ') {
                workingString += Character.toString(newbyte);
                //System.out.println("LINE 43 " + workingString);
            } else  {
                if  (!workingString.contains("-")) {
                    //System.out.println("LINE 46 " + workingString);
                    workingString = "";
                    continue;
                }
                Matcher lidarBlob  = lidarFormat.matcher(workingString);

                if (!lidarBlob.matches()) {
                    //System.out.println("LINE 52 " + workingString);
                    workingString = "";
                    continue;
                }

                //System.out.println("LINE 56 " + workingString);
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
        SmartDashboard.putNumber("Lidar 0: ", getValue(0));
    }

    public Integer getValue(int id) {
        if (lidars.containsKey(id)) {
            return lidars.get(id);
        } else {
            return null;
        }
    }
}
