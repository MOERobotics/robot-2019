package frc.robot.autonomous.sandstorm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.autonomous.GenericAuto;

public class MAShipFrontHatch1ReloadAuto extends GenericAuto  {
    PIDModule MOErioAuto = new PIDModule(0.06, 0.001, 0);
    //PIDModuleLucy MOErioTurn = new PIDModuleLucy(2.5e-2, 1.75e-3, 0);
    long startTime = 0;
    double z = 1.33;
    double louWizardry = 0;

    //-1 is left, 1 is right
    int turncounter = 0;
    double correction = 0;
    double moementumCorrection = 100;
    double zEffective;

    int approachHeading = 8;

    PIDModule elevatorPID = new PIDModule(0.1, 0.00, 0);
    PIDModule armPID = new PIDModule(1.75e-2,3.0e-3,0);
    double elevatorCorrection;
    double armCorrection;
    double armPowerBias = 0;
    double elevatorDeploy = 13.1;
    double elevatorFloor = -30/*-3.13*/;
    double armOut = 20;

    int midPoint = 34;
    int topXVal;

    int margin = 2;
    int biggerMargin = 6;
    double turnPower = 0.2;
    double higherTurnPower = 0.25;

    int numTimesNull = 0;
    int pixyWait = 0; //frame counter for waiting between pixy adjustments

    //case 7, bonus begins and normal auto ends

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

        levelTwo = false;
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

            /*raise elevator*/
            case 0:
                robot.stopDriving();
                robot.driveElevator(0.8);
                if(robot.getElevatorEncoderCount()  >= elevatorDeploy){
                    autoStep++;
                    elevatorPID.resetError();
                }
                break;

            /*raise arm*/
            case 1:
                robot.stopDriving();
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorDeploy);
                elevatorCorrection = elevatorPID.getCorrection();

                robot.driveElevator(elevatorCorrection);

                robot.driveArm(0.2);
                if (robot.getArmEncoderCount()  >= armOut){
                    armPID.resetError();
                    autoStep = 3;
                }
                break;

            //keep em still
            case 2:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorDeploy);
                elevatorCorrection = elevatorPID.getCorrection();

                robot.driveElevator(elevatorCorrection);

                armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                armCorrection = armPID.getCorrection();

                robot.driveArm(armPowerBias + armCorrection);
                MOErioAuto.resetError();
                autoStep++;
                break;

            //drive toward cargo ship
            case 3:
                MOErioAuto.setHeading(robot.getHeadingDegrees() - approachHeading * LeftSide);
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower(0.4 * (1 + correction),
                        0.4 * (1 - correction));

                    if (robot.getDistanceLeftInches() > 64) {
                    autoStep++;
                }
                break;

            //auto targeting
            case 4:
                elevatorPID.setHeading(robot.getElevatorEncoderCount() - elevatorDeploy);
                elevatorCorrection = elevatorPID.getCorrection();

                robot.driveElevator(elevatorCorrection);

                armPID.setHeading(robot.getArmEncoderCount() - armOut);
                armCorrection = armPID.getCorrection();

                robot.driveArm(armPowerBias + armCorrection);

                if(pixyWait < 5){ pixyWait++; break; }
                pixyWait = 0;
                if (robot.pixy.vec.length != 1) {
                    //Null counter, if not detecting pixy lines, don't move
                    numTimesNull++;
                    if (numTimesNull > 4){
                        robot.setDrivePower(0, 0);
                    }
                } else {
                    numTimesNull = 0; //reset null exit counter
                    topXVal = robot.pixy.vec[0].getX1();

                    //If top vec coord is to far to right
                    if (topXVal > midPoint + margin) {
                        //If really close, move less
                        if (topXVal > midPoint + biggerMargin) {
                            robot.setDrivePower(turnPower, 0);
                        } else {
                            robot.setDrivePower(higherTurnPower, 0);
                        }
                    } else if (topXVal < midPoint - margin) {
                        if (topXVal < midPoint - biggerMargin) { //big margin because line moves farther, the closer robot is
                            robot.setDrivePower(0, turnPower);
                        } else {
                            robot.setDrivePower(0, higherTurnPower);
                        }
                    }

                    if (Math.abs(topXVal - midPoint) <= margin){
                        autoStep++;
                        startTime = System.currentTimeMillis();
                    }
                }
                break;

            //lower elevator
            case 5:
                armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                armCorrection = armPID.getCorrection();

                robot.driveArm(armPowerBias + armCorrection);

                robot.driveElevator(-0.3);
                if(robot.getElevatorEncoderCount()  <= elevatorFloor){
                    autoStep++;
                }
                break;

            //spear out, drive forward
            case 6:
                robot.spearOut();
                robot.setDrivePower(0.3,0.3);
                if(robot.lidar[0] < 475){
                    autoStep++;
                    startTime = System.currentTimeMillis();
                }
                break;

            //drive forward one second
            case 7:
                robot.setDrivePower(0.2,0.2);
                if(System.currentTimeMillis() - 1000 > startTime){
                    autoStep++;
                }
                break;

            //fingers out
            case 8:
                robot.spearHook();
                robot.stopDriving();
                autoStep++;
                break;


            case 9:
                robot.setDrivePower(-0.3,-0.3);//post-Lehigh: What is this supposed to do?
                if(robot.lidar[0] < 500){
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                }
                break;

            case 10:
                //fingers in, spear in
                robot.spearIn();
                robot.spearUnhook();
                //roll backwards
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower(-0.3 * (1 + correction),-0.3 * (1 - correction));
                if(leftDistance > 12){ //this distance needs to be confirmed
                    autoStep++;
                }
                break;

            case 11:
                //turn around.
                robot.setDrivePower(0.2,-0.2);
                if(robot.getHeadingDegrees() > 125){//this heading needs to be confirmed
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                }
                break;

            case 12:
                //roll forward a lot towards the loading station (distance needs to be confirmed)
                MOErioAuto.setHeading(robot.getHeadingDegrees() - 135);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(0.3*(1+correction), 0.3*(1-correction));
                if(leftDistance > 96){
                    autoStep++;
                }
                break;

            case 13:
                //straighten using pixy align
                if(pixyWait < 5){ pixyWait++; break; }
                pixyWait = 0;
                if (robot.pixy.vec.length != 1) {
                    //Null counter, if not detecting pixy lines, don't move
                    numTimesNull++;
                    if (numTimesNull > 4){
                        robot.setDrivePower(0, 0);
                    }
                } else {
                    numTimesNull = 0; //reset null exit counter
                    topXVal = robot.pixy.vec[0].getX1();

                    //If top vec coord is to far to right
                    if (topXVal > midPoint + margin) {
                        //If really close, move less
                        if (topXVal > midPoint + biggerMargin) {
                            robot.setDrivePower(turnPower, 0);
                        } else {
                            robot.setDrivePower(higherTurnPower, 0);
                        }
                    } else if (topXVal < midPoint - margin) {
                        if (topXVal < midPoint - biggerMargin) { //big margin because line moves farther, the closer robot is
                            robot.setDrivePower(0, turnPower);
                        } else {
                            robot.setDrivePower(0, higherTurnPower);
                        }
                    }

                    if (Math.abs(topXVal - midPoint) <= margin){
                        autoStep++;
                        startTime = System.currentTimeMillis();
                    }
                }
                break;

            case 14:
                //now roll forward
                robot.setDrivePower(0.3,0.3);
                if(robot.lidar[0]<475){//this needs to be confirmed
                    autoStep++;
                }
                break;

            case 15:
                robot.stopDriving();
                break;

        }
    }

}
