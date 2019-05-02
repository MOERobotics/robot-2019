package frc.robot.autonomous.unused;

import frc.robot.PIDModule;
import frc.robot.autonomous.GenericAuto;

public class AutoTargetTest extends GenericAuto {

        PIDModule elevatorPID = new PIDModule(0.1, 0.00, 0);
        PIDModule armPID = new PIDModule(1.75e-2, 3.0e-3, 0);
        double elevatorCorrection;
        double armCorrection;
        double armPowerBias = 0;
        double elevatorDeploy = 13.1;
        double elevatorFloor = -28.6 - 3.13;
        double armOut = /*21.3*/ 21.0;
        double elevatorPeek = -28.6 - 3.13 + (9*2.85);
        int currentLidar;
        int VisualCenter = 150;
        int DesiredVisualCenter = 25;

        public int Peek(double d) {
            return 100;
        }

        @Override
        public void init () {
            autoStep = 1;
            elevatorPID.resetError();
            armPID.resetError();

        }

        @Override
        public void printSmartDashboard () {
        }

        @Override
        public void run () {

            switch (autoStep) {
                case 1:
                    /*elevatorPID.setHeading(robot.getElevatorEncoderCount() - elevatorDeploy);
                    elevatorCorrection = elevatorPID.getCorrection();

                    robot.driveElevator(elevatorCorrection);*/
                    if (robot.getArmEncoderCount() > armOut + 5) {
                        robot.driveArm(-0.2);
                    } else if (robot.getArmEncoderCount() < armOut - 5) {
                        robot.driveArm(0.2);
                    } else {
                        armPID.resetError();
                        autoStep++;
                    }

                    break;

                case 2:
                    armPID.setHeading(robot.getArmEncoderCount() - armOut);
                    armCorrection = armPID.getCorrection();

                    robot.driveArm(armPowerBias + armCorrection);

                    if (robot.getElevatorEncoderCount() > elevatorPeek + 5) {
                        robot.driveElevator(-0.3);
                    } else if (robot.getElevatorEncoderCount() < elevatorPeek - 5) {
                        robot.driveElevator(0.3);
                    } else {
                        elevatorPID.resetError();
                        autoStep++;
                    }

                    break;

                case 3:
                    elevatorPID.setHeading(robot.getElevatorEncoderCount() - elevatorPeek);
                    elevatorCorrection = elevatorPID.getCorrection();

                    robot.driveElevator(elevatorCorrection);

                    armPID.setHeading(robot.getArmEncoderCount() - armOut);
                    armCorrection = armPID.getCorrection();

                    robot.driveArm(armPowerBias + armCorrection);

                    /*VisualCenter = robot.piXY[0];
                    currentLidar = robot.lidar[0];
                    DesiredVisualCenter = Peek(currentLidar);*/
                    autoStep++;
                    break;

                case 4:
                    //VisualCenter = robot.piXY[0];
                    if (VisualCenter != -1)
                    {
                        if (VisualCenter-DesiredVisualCenter > 5) {
                            /* Rotate clockwise */
                            robot.turnRightInplace(0.1);
                        } else if (VisualCenter - DesiredVisualCenter < -5) {
                            /* Rotate anti-clockwise */
                            robot.turnLeftInplace(0.1);
                        } else {
                            robot.setDrivePower(0,0);
                        }
                    }

                    break;

            }

        }

}
