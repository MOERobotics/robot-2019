package frc.robot.autonomous.test;

import frc.robot.autonomous.GenericAuto;

public class AutoTest extends GenericAuto {
    public static long endTime;
    public void init() {
        autoStep = -1;
    }
    public void run() {
        switch (autoStep) {
            //case 0 move forward 2 feet
            case -1:

                robot.resetDriveEncoders();
                robot.resetYaw();
                autoStep = 0;
            case 0:
                robot.moveForward(0.25);
                if (robot.getDistanceLeftInches() >= 24) {
                    autoStep = 1;
                    long startTime = System.currentTimeMillis();
                    endTime =  startTime + 1000;
                }
                break;
                //case 1 stops for one second
            case 1:
                robot.stopDriving();
                if (System.currentTimeMillis() >= endTime) {
                    autoStep = 2;
                    robot.resetDriveEncoders();
                }
                break;
                //case 2 moves backwards 2 feet
            case 2:
                robot.moveBackward(0.25);
                if (robot.getDistanceLeftInches() <= -24) {
                    autoStep = 3;
                    long startTime = System.currentTimeMillis();
                    endTime =  startTime + 1000;
                }
                break;
            //case 3 stops for one second
            case 3:
                robot.stopDriving();
                if (System.currentTimeMillis() >= endTime) {
                    autoStep = 4;
                    robot.resetYaw();
                }
                break;
                //case 4 turns left 90 degrees
            case 4:
                robot.turnLeftInplace(0.3);
                if (robot.getHeadingDegrees() <= -90) {
                    autoStep = 5;
                    long startTime = System.currentTimeMillis();
                    endTime =  startTime + 1000;
                }
                break;
                //case 5 stops for one second
            case 5:
                robot.stopDriving();
                if (System.currentTimeMillis() >= endTime) {
                    autoStep = 6;
                    robot.resetYaw();
                }
                break;
                //case 6 turns right 180 degrees
            case 6:
                robot.turnRightInplace(0.3);
                if (robot.getHeadingDegrees() >= 178) {
                    autoStep = 7;
                    long startTime = System.currentTimeMillis();
                    endTime =  startTime + 1000;
            }
                break;
                //case 7 stops for one second
            case 7:
                robot.stopDriving();
                if (System.currentTimeMillis() >= endTime) {
                    autoStep = 8;
                    robot.resetYaw();
                }
                break;
                //case 8 turns 90 degrees
            case 8:
                robot.turnLeftInplace(0.3);
                if (robot.getHeadingDegrees() <= -90) {
                    autoStep = 9;
                    long startTime = System.currentTimeMillis();
                    endTime =  startTime + 1000;
                }
                break;
            case 9:
                robot.stopEverything();
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