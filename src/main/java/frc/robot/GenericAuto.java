package frc.robot;

import frc.robot.genericrobot.GenericRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class GenericAuto {
    public int autoStep = 0;
    public abstract void init();
    public abstract void run();
    public GenericRobot robot;

    private boolean haveWeYelledAtTheCoderYet = false;
    public void printSmartDashboard() {

        if(haveWeYelledAtTheCoderYet == false) {
            System.err.println("Help us God");
            haveWeYelledAtTheCoderYet = true;
        }
    }
}
