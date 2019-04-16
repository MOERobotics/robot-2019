package frc.robot;
import edu.wpi.first.wpilibj.SerialPort;

import java.util.HashMap;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class LidarReader extends Thread{
    private Map<Integer, Integer> lidars = new HashMap<>(8);
    SerialPort lidarPort;

    public LidarReader (SerialPort lidarPort) {
        this.lidarPort = lidarPort;
    }

    private String workingString = " ";
    private Pattern lidarFormat = Pattern.compile("(?<id>\\d+)-(?<value>\\d+)");

    @Override
    public void run() {
        while (true) {
            byte newbyte = lidarPort.read(1)[0];

            if (newbyte != ' ') {
                workingString += Character.toString(newbyte);
            } else  {
                if  (!workingString.contains("-")) {
                    continue;
                }
                Matcher lidarBlob  = lidarFormat.matcher(workingString);

                if (!lidarBlob.matches()) {
                    continue;
                }

                int id = Integer.parseInt((lidarBlob.group(   "id")));
                int value = Integer.parseInt(lidarBlob.group("value"));

                lidars.put(id,value);
            }
        }
    }

    public Integer getValue(int id) {
        if (lidars.containsKey(id)) {
            return lidars.get(id);
        } else {
            return null;
        }
    }
}
