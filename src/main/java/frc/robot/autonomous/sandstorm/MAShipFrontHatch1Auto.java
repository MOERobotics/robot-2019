package frc.robot.autonomous.sandstorm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.autonomous.GenericAuto;

public class MAShipFrontHatch1Auto extends GenericAuto  {
    long startTime = 0;

    //arc PID
    //-1 is left, 1 is right
    PIDModule MOErioAuto = new PIDModule(0.06, 0.001, 0);
    //PIDModuleLucy MOErioTurn = new PIDModuleLucy(2.5e-2, 1.75e-3, 0);
    double z = 1.33;
    double louWizardry = 0;
    double correction = 0;
    double zEffective;

    //unused
    double moementumCorrection = 100;
    int turncounter = 0;
    double orientationTolerance = 0.5;

    //constants
    boolean levelTwo = false;
    int approachHeading = 8;

    //elevator and arm PID
    PIDModule elevatorPID = new PIDModule(0.1, 0.00, 0);
    PIDModule armPID = new PIDModule(1.75e-2,3.0e-3,0);
    double elevatorDeploy = 13.1;
    double elevatorFloor = -30/*-3.13*/;
    double armOut = 20;

    //pixy constants
    int midPoint = 34;
    int topXVal;
    int margin = 2;
    int biggerMargin = 6;
    double turnPower = 0.2/*0.4*/;
    double higherTurnPower = 0.25/*0.45*/;
    int numTimesNull = 0;
    int pixyWait = 0; //frame counter for waiting between pixy adjustments

    @Override
    public void init() {
        autoStep = -2;
        //autoStep = 9;
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

        withinElevatorTolerance = false;
        withinArmTolerance = false;
    }

    @Override
    public void printSmartDashboard() {
        /*
        correction = MOErioAuto.getCorrection();
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
                robot.setDrivePower((0.4) * (1 + correction), (0.4) * (1 - correction));

                if (levelTwo) {
                    if (leftDistance >= 48 * 2) {
                        robot.resetDriveEncoders();
                        MOErioAuto.resetError();
                        autoStep++;
                        robot.stopDriving();
                    }
                } else {
                    if (leftDistance >= 48) {
                        robot.resetDriveEncoders();
                        MOErioAuto.resetError();
                        autoStep++;
                        robot.stopDriving();
                    }
                }
                break;

            //raise elevator
            case 0:
                robot.stopDriving();
                if (!withinElevatorTolerance) {
                    raiseElevator(elevatorDeploy, elevatorPID);
                } else{
                    autoStep++;
                }
                break;

            //raise arm
            case 1:
                robot.stopDriving();
                PIDElevator(elevatorDeploy, elevatorPID);

                if(!withinArmTolerance){
                    raiseArm(armOut, armPID);
                } else {
                    autoStep++;
                }

                break;

            //keep still
            case 2:
                PIDElevator(elevatorDeploy, elevatorPID);
                PIDArm(armOut, armPID);

                MOErioAuto.resetError();
                autoStep++;
                break;

            //drive forward ~64 inches
            case 3:
                MOErioAuto.setHeading(robot.getHeadingDegrees() - approachHeading * LeftSide);
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower(0.4 * (1 + correction),
                        0.4 * (1 - correction));

                if (robot.getDistanceLeftInches() > 64) {
                    autoStep++;
                }
                break;

            //auto align
            case 4:
                PIDElevator(elevatorDeploy, elevatorPID);
                PIDArm(armOut, armPID);

                if (robot.pixy.vec.length != 1) {
                    //Null counter, if not detecting pixy lines, don't move
                    numTimesNull++;
                    if (numTimesNull > 4){
                        robot.setDrivePower(0, 0);
                    }
                } else {
                    numTimesNull = 0; //reset null exit counter

                    if (robot.pixy.vec.length != 0 && robot.pixy.vec[0] != null) {
                        //which point of vector is higher on screen? get that point's X val
                        topXVal = robot.pixy.vec[0].getX1();
                        if (robot.pixy.vec[0].getY0() < robot.pixy.vec[0].getY1()) {
                            topXVal = robot.pixy.vec[0].getX0();
                        }
                    }

                    if(pixyWait < 5){ pixyWait++; break; }
                    pixyWait = 0;

                    if (topXVal > midPoint + margin) {
                        if (topXVal > midPoint + biggerMargin) {
                            robot.setDrivePower(higherTurnPower,-higherTurnPower);
                        } else {
                            robot.setDrivePower(turnPower, -turnPower);
                        }
                    } else if (topXVal < midPoint - margin) {
                        if (topXVal < midPoint - biggerMargin) {
                            robot.setDrivePower(-higherTurnPower,higherTurnPower);
                        } else {
                            robot.setDrivePower(-turnPower, turnPower);
                        }
                    } else {
                        autoStep++;
                        robot.stopDriving();
                    }
                }
                break;

            //lower elevator
            case 5:
                PIDArm(armOut, armPID);
                robot.driveElevator(-0.3);

                if(robot.getElevatorEncoderCount()  <= elevatorFloor+3){
                    autoStep++;
                    elevatorPID.resetError();
                }
                break;

            //out the spear and drive forward (may need to pid the elevator in a preceding step)
            case 6:
                PIDArm(armOut, armPID);
                PIDElevator(elevatorFloor + 3, elevatorPID);//in order to give us more leeway from the pegs.
                robot.spearOut();
                robot.setDrivePower(0.3,0.3);
                if(robot.lidar[0] < 475){
                    autoStep++;
                    startTime = System.currentTimeMillis();
                }
                break;

            //drive forward for 1 second
            case 7:
                robot.setDrivePower(0.2,0.2);
                if(System.currentTimeMillis() - 1000 > startTime){
                    autoStep++;
                }
                break;

            //fingers in
            case 8:
                robot.spearHook();
                //robot.spearOut();
                robot.stopDriving();
                autoStep++;
                break;

            //spear in, drive backwards for some reason.
            case 9:
                robot.spearIn();
                robot.setDrivePower(-0.3,-0.3);
                if(robot.lidar[0] < 500){
                    autoStep++;
                }
                break;

            //done.
            case 10:
                robot.stopDriving();
                break;

        }
    }


    //48 in, 60/50 deg, elevator up, arm up, elevator down, 86 inches, autoapproach

}
