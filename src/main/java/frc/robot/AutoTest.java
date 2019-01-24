package frc.robot;

import frc.robot.genericrobot.GenericRobot;

public class AutoTest {
    public static int autoStep = 0;
    public static long endTime;
    public static void init() {
        autoStep = -1;
    }
    public static void run(GenericRobot Robot) {
        switch (autoStep) {
            //case 0 move forward 2 feet
            case -1:

                Robot.resetDriveEncoder();
                Robot.resetYaw();
                autoStep = 0;
            case 0:
                Robot.moveForward(0.25);
                if (Robot.getDistanceLeftInches() >= 24) {
                    autoStep = 1;
                    long startTime = System.currentTimeMillis();
                    endTime =  startTime + 1000;
                }
                break;
                //case 1 stops for one second
            case 1:
                Robot.stopDriving();
                if (System.currentTimeMillis() >= endTime) {
                    autoStep = 2;
                    Robot.resetDriveEncoder();
                }
                break;
                //case 2 moves backwards 2 feet
            case 2:
                Robot.moveBackward(0.25);
                if (Robot.getDistanceLeftInches() <= -24) {
                    autoStep = 3;
                    long startTime = System.currentTimeMillis();
                    endTime =  startTime + 1000;
                }
                break;
            //case 3 stops for one second
            case 3:
                Robot.stopDriving();
                if (System.currentTimeMillis() >= endTime) {
                    autoStep = 4;
                    Robot.resetYaw();
                }
                break;
                //case 4 turns left 90 degrees
            case 4:
                Robot.turnLeftInplace(0.3);
                if (Robot.getHeadingDegrees() <= -90) {
                    autoStep = 5;
                    long startTime = System.currentTimeMillis();
                    endTime =  startTime + 1000;
                }
                break;
                //case 5 stops for one second
            case 5:
                Robot.stopDriving();
                if (System.currentTimeMillis() >= endTime) {
                    autoStep = 6;
                    Robot.resetYaw();
                }
                break;
                //case 6 turns right 180 degrees
            case 6:
                Robot.turnRightInplace(0.3);
                if (Robot.getHeadingDegrees() >= 180) {
                    autoStep = 7;
                    long startTime = System.currentTimeMillis();
                    endTime =  startTime + 1000;
            }
                break;
                //case 7 stops for one second
            case 7:
                Robot.stopDriving();
                if (System.currentTimeMillis() >= endTime) {
                    autoStep = 7;
                    Robot.resetYaw();
                }
                break;
                //case 8 turns 90 degrees
            case 8:
                Robot.turnLeftInplace(0.3);
                if (Robot.getHeadingDegrees() <= -90) {
                    autoStep = 9;
                    long startTime = System.currentTimeMillis();
                    endTime =  startTime + 1000;
                }
                break;
            case 9:
                Robot.stopEverything();
        }
    }
}

/*

Move forward
Stop at 2 feet

Stop moving
Wait one second

Move backwards
Stop at 2 feet

Stop moving
Wait one second

Turn Left
Stop at 90 degrees

Stop moving
Wait one second

Turn Right
Stop at 180 degrees

Stop moving
Wait one second

Turn Left
Stop at 90 degrees

Stop moving

 */