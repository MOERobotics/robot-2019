package frc.robot.autonomous.test;

import frc.robot.autonomous.GenericAuto;

public class UnitTestElevArmPos extends GenericAuto {

    @Override
    public void init() {
        autoStep = 0;
    }

    @Override
    public void run() {

        switch (autoStep) {
            /*case 0:
                if (robot.getElevatorEncoderCount() <= (robot.getElevatorOrigin() - 19) &&
                        robot.getElevatorEncoderCount() >= (robot.getElevatorOrigin() - 24)) {
                    autoStep++;
                } else {
                    if (robot.getElevatorEncoderCount() > (robot.getElevatorOrigin() - 22.5))
                        robot.driveElevator(-0.3);
                    else if (robot.getElevatorEncoderCount() < (robot.getElevatorOrigin() - 22.5))
                        robot.driveElevator(0.3);
                }
                break;
            case 1:
                robot.driveElevator(0);
                if (robot.getElevatorPower() == 0) autoStep++;
                break;
            case 2:
                if (robot.getArmEncoderCount() <= (robot.getArmOrigin() - 19) &&
                        robot.getElevatorEncoderCount() >= (robot.getElevatorOrigin() - 24)) {
                    autoStep++;
                } else {
                    if (robot.getElevatorEncoderCount() > (robot.getElevatorOrigin() - 22.5))
                        robot.driveElevator(-0.3);
                    else if (robot.getElevatorEncoderCount() < (robot.getElevatorOrigin() - 22.5))
                        robot.driveElevator(0.3);
                }
                break;
            */
        }

    }
}
