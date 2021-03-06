package frc.robot.autonomous.sandstorm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.PIDModuleLucy;
import frc.robot.autonomous.GenericAuto;
import frc.robot.genericrobot.MOErio;

public class MASideAutoPixy extends GenericAuto {
    PIDModule MOErioAuto = new PIDModule(0.06, 0.001, 0);
    //PIDModuleLucy MOErioTurn = new PIDModuleLucy(2.5e-2, 1.75e-3, 0);
    long startTime = 0;
    double louWizardry = 0;

    //-1 is left, 1 is right
    int turncounter = 0;
    double correction = 0;
    double zEffective;

    PIDModule elevatorPID = new PIDModule(0.1, 0.00, 0);
    PIDModule armPID = new PIDModule(1.75e-2,3.0e-3,0);
    double elevatorCorrection;
    double armCorrection;
    double armPowerBias = 0;
    double elevatorDeploy = 13.1;
    double elevatorFloor = -28.6/*-3.13*/;
    double armOut = 59;


    int topXVal;
    int numTimesNull = 0;

    @Override
    public void init() {
        autoStep = -2;
        robot.resetDriveEncoders();
        robot.resetYaw();
        MOErioAuto.resetError();
        MOErioAuto.setHeading(0);
        //MOErioTurn.resetError();
        startTime = System.currentTimeMillis();

        if (LeftSide == -1) {
            zEffective = 1 / z;
        } else {
            zEffective = z;
        }
    }

    @Override
    public void printSmartDashboard() {
        /*correction = MOErioAuto.getCorrection();
        SmartDashboard.putNumber("Error: ", MOErioAuto.getInput());
        SmartDashboard.putNumber("Correction: ", correction);
        SmartDashboard.putNumber("kP: ", MOErioAuto.pidController.getP());
        SmartDashboard.putNumber("kI: ", MOErioAuto.pidController.getI());
        SmartDashboard.putNumber("kD: ", MOErioAuto.pidController.getD());
        SmartDashboard.putNumber("Current Time: ", System.currentTimeMillis());
        SmartDashboard.putNumber("The Magic: ", louWizardry);
        SmartDashboard.putNumber("Z: ", z);
        SmartDashboard.putNumber("Abs Left", Math.abs(robot.getDistanceLeftInches()));
        SmartDashboard.putNumber("Abs Right", Math.abs(robot.getDistanceRightInches()));
        SmartDashboard.putNumber("Left Side", LeftSide);
        SmartDashboard.putBoolean("Level Two", levelTwo);
        */
    }

    @Override
    public void run() {
        SmartDashboard.putNumber("autoStep", autoStep);
        double leftDistance = Math.abs(robot.getDistanceLeftInches());
        double rightDistance = Math.abs(robot.getDistanceRightInches());
        louWizardry = leftDistance - rightDistance * zEffective;

        switch (autoStep) {
            case -2:
                MOErioAuto.resetError();
                robot.resetYaw();
                robot.resetDriveEncoders();

                if (System.currentTimeMillis() >= startTime + 100) {
                    MOErioAuto.resetError();
                    robot.resetYaw();
                    robot.resetDriveEncoders();
                    autoStep++;
                }
                break;

            /*drive off the HAB*/
            case -1:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();

                //correction negative, left motor decrease, correction positive, left motor power increase
                robot.setDrivePower((0.8) * (1 + correction), (0.8) * (1 - correction));

                if (levelTwo) {
                    if (leftDistance >= 48 * 2) {
                        robot.resetDriveEncoders();
                        MOErioAuto.resetError();
                        autoStep++;
                    }
                } else {
                    if (leftDistance >= 48) {
                        robot.resetDriveEncoders();
                        MOErioAuto.resetError();
                        autoStep++;
                    }
                }
                break;

            /*Right side- make a right turning arc*/
            /*Left side- make a left turning arc*/
            case 0:
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                setDrivePowerHands(/*0.5*/0.7,/*0.3*/0.5, correction, LeftSide);

                if (getDistanceLeftInchesHands(LeftSide) >= 72 /*x1*/) {
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                }
                break;

            /*Right side- make a left turning arc*/
            /*Left side- make a right turning arc*/
            /*you should be sideways to the cargo ship*/
            case 1:
                louWizardry = leftDistance - rightDistance / zEffective;
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();
                setDrivePowerHands(0.5, 0.7, correction, LeftSide);


                if (robot.getElevatorEncoderCount() < elevatorDeploy) {
                    robot.driveElevator(0.6);
                } else {
                    robot.driveElevator(0);
                }

                if (getDistanceRightInchesHands(LeftSide) >= 43 /*x2*/) {
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                }
                break;

            /*roll forward, now in front of hatch*/
            case 2:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower((0.4) * (1 + correction), (0.4) * (1 - correction));

                if (robot.getElevatorEncoderCount() < elevatorDeploy) {
                    robot.driveElevator(0.6);
                } else {
                    robot.driveElevator(0);
                }

                if (leftDistance >= 49) {
                    autoStep++;
                    MOErioAuto.resetError();
                    //MOErioTurn.resetError();
                    elevatorPID.resetError();
                }
                break;

            /*Right side- turn 90 degrees to the left*/
            /*Left side- turn 90 degrees to the right*/
            /*turning towards the hatch*/
            case 3:
                elevatorPID.setHeading(robot.getElevatorEncoderCount() - elevatorDeploy);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                robot.setDrivePower(-0.5 * LeftSide, 0.5 * LeftSide);

                if (reachedHeadingHands(80, -1 * LeftSide)) {
                    //MOErioTurn.resetError();
                    robot.setDrivePower(0, 0);
                    autoStep++;
                }
                break;

            /*turning cont'd*/

            case 4:
                autoStep++;
                break;

            case 5:
                elevatorPID.setHeading(robot.getElevatorEncoderCount() - elevatorDeploy);
                elevatorCorrection = elevatorPID.getCorrection();

                robot.driveElevator(elevatorCorrection);

                robot.driveArm(0.6);
                if (robot.getArmEncoderCount() >= armOut - 5) {
                    armPID.resetError();
                    autoStep++;
                }
                break;

            case 6:
                armPID.setHeading(robot.getArmEncoderCount() - armOut);
                armCorrection = armPID.getCorrection();

                robot.driveArm(armPowerBias + armCorrection);

                robot.driveElevator(-0.4);
                if (robot.getElevatorEncoderCount() <= elevatorFloor + 2) {
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                    turncounter = 0;
                }
                break;

            /*roll towards the hatch*/
            case 7:
                elevatorPID.setHeading(robot.getElevatorEncoderCount() - elevatorFloor);
                elevatorCorrection = elevatorPID.getCorrection();

                robot.driveElevator(elevatorCorrection);

                armPID.setHeading(robot.getArmEncoderCount() - armOut);
                armCorrection = armPID.getCorrection();

                robot.driveArm(armPowerBias + armCorrection);

                MOErioAuto.setHeading(robot.getHeadingDegrees() - -90 * LeftSide);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower((0.4) * (1 + correction), (0.4) * (1 - correction));

                if (turncounter < 5) {
                    if (robot.pixy.vec.length > 0) {
                        turncounter++;
                    } else {
                        turncounter = 0;
                    }
                } else {
                    autoStep++;
                    robot.setDrivePower(0, 0);
                    turncounter = 0;
                }
                break;

            case 8:
                elevatorPID.setHeading(robot.getElevatorEncoderCount() - elevatorFloor);
                elevatorCorrection = elevatorPID.getCorrection();

                robot.driveElevator(elevatorCorrection);

                armPID.setHeading(robot.getArmEncoderCount() - armOut);
                armCorrection = armPID.getCorrection();

                robot.driveArm(armPowerBias + armCorrection);

                if (robot.pixy.vec.length != 1) {
                    numTimesNull++;
                    if (numTimesNull > 4){
                        robot.setDrivePower(0, 0);
                    }
                } else {
                    numTimesNull = 0; //reset null exit counter
                    topXVal = robot.pixy.vec[0].getX1();

                    if (topXVal > midPoint + margin) {
                        if (topXVal > midPoint + biggerMargin) {
                            robot.setDrivePower(higherTurnPower, -higherTurnPower);
                        } else {
                            robot.setDrivePower(turnPower, -turnPower);
                        }
                    } else if (topXVal < midPoint - margin) {
                        if (topXVal < midPoint - biggerMargin) {
                            robot.setDrivePower(-higherTurnPower, higherTurnPower);
                        } else {
                            robot.setDrivePower(-turnPower, turnPower);
                        }
                    }

                    if (Math.abs(topXVal - midPoint) <= margin){
                        autoStep++;
                        startTime = System.currentTimeMillis();
                    }
                }
                break;

            case 9:
                elevatorPID.setHeading(robot.getElevatorEncoderCount() - elevatorFloor);
                elevatorCorrection = elevatorPID.getCorrection();

                robot.driveElevator(elevatorCorrection);

                armPID.setHeading(robot.getArmEncoderCount() - armOut);
                armCorrection = armPID.getCorrection();

                robot.driveArm(armPowerBias + armCorrection);

                MOErioAuto.setHeading(robot.getHeadingDegrees() + 90 * LeftSide);
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower(0.3 * (1 + correction), 0.3 * (1 - correction));

                if ((robot.lidar[0] <= 850) || (System.currentTimeMillis() - startTime > 500)) { //750
                    autoStep++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 10:
                if (System.currentTimeMillis() - startTime > 500) {
                    autoStep++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 11:
                elevatorPID.setHeading(robot.getElevatorEncoderCount() - elevatorFloor);
                elevatorCorrection = elevatorPID.getCorrection();

                robot.driveElevator(elevatorCorrection);

                armPID.setHeading(robot.getArmEncoderCount() - armOut);
                armCorrection = armPID.getCorrection();

                robot.driveArm(armPowerBias + armCorrection);

                if (System.currentTimeMillis() - startTime > 750) {
                    robot.rollOut(0);
                    robot.driveArm(0);
                    robot.stopDriving();
                    robot.resetDriveEncoders();
                    autoStep++;
                } else {
                    robot.rollOut(0.5);
                    robot.stopDriving();
                }
                break;

            case 12:
                MOErioAuto.setHeading(robot.getHeadingDegrees() + 90 * LeftSide);
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower(-0.4 * (1 - correction), -0.4 * (1 + correction));

                if (Math.abs(robot.getDistanceLeftInches()) > 12) {
                    robot.driveArm(-0.4);
                    if (robot.getArmEncoderCount() < 10) robot.driveArm(0);
                }

                if (Math.abs(robot.getDistanceLeftInches()) > 32) {
                    autoStep++;
                }
                break;

            case 13:
                robot.setDrivePower(-0.4 * LeftSide, 0.4 * LeftSide);

                if ((LeftSide == 1 && robot.getHeadingDegrees() < -160)
                    || (LeftSide == -1 && robot.getHeadingDegrees() > 160)
                ) {
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                }
                break;

            case 14:
                MOErioAuto.setHeading(Math.sin(
                        ((robot.getHeadingDegrees()-180)
                                * Math.PI / 180.0))
                );

                correction = MOErioAuto.getCorrection();
                robot.setDrivePower(0.8 * (1 + correction), 0.8 * (1 - correction));

                if (Math.abs(robot.getDistanceLeftInches()) > 144) {
                    robot.stopDriving();
                    autoStep++;
                }
                break;

            case 15:
                robot.stopDriving();
                break;
        }
    }
}
