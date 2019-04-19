package frc.robot.autonomous.sandstorm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.autonomous.*;

public class MARocketHatch1Auto extends GenericAuto  {
    long startTime = 0;

    //arc PID
    PIDModule MOErioAuto = new PIDModule(0.06, 0.001, 0);
    //PIDModuleLucy MOErioTurn = new PIDModuleLucy(2.5e-2, 1.75e-3, 0);

    //-1 is left, 1 is right
    double z = 1.33;
    double louWizardry = 0;
    double zEffective;
    double correction = 0;

    //unused
    int turncounter = 0;
    double moementumCorrection = 100;

    //position and power variables
    boolean levelTwo = false;
    int approachHeading = 40+5;

    //elevator and arm PID
    PIDModule elevatorPID = new PIDModule(0.1, 0.00, 0);
    PIDModule armPID = new PIDModule(1.75e-2,3.0e-3,0);
    double elevatorDeploy = 13.1;
    double elevatorFloor = -30/*-3.13*/;
    double armOut = /*19*/20;

    //pixy constants + variables
    int midPoint = 34;
    int margin = 1;
    int biggerMargin = 8;
    int topXVal;
    int numTimesNull = 0;
    int pixyWait = 0;

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

        robot.shiftHigh();

        withinElevatorTolerance = false;
        withinArmTolerance = false;
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
        SmartDashboard.putBoolean("Level Two", levelTwo);*/
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

            /*drive off the HAB and raise elevator*/
            case -1:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();

                //correction negative, left motor decrease, correction positive, left motor power increase
                robot.setDrivePower((0.4) * (1 + correction), (0.4) * (1 - correction));

                if(!withinElevatorTolerance){
                    raiseElevator(elevatorDeploy, elevatorPID);
                }else{
                    PIDElevator(elevatorDeploy,elevatorPID);
                }

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

            /*continue to raise the elevator*/
            case 0:
                if(!withinElevatorTolerance){
                    raiseElevator(elevatorDeploy, elevatorPID);
                }else{
                    PIDElevator(elevatorDeploy,elevatorPID);
                    autoStep++;
                }
                break;


            /*keep still*/
            case 1:
                PIDElevator(elevatorDeploy, elevatorPID);

                autoStep++;
                break;

            /*turn towards the rocket*/
            case 2:
                /* LFR */
                if (reachedHeadingHands(approachHeading, LeftSide)) {
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                }
                robot.setDrivePower(0.5*LeftSide, -0.5*LeftSide);
                break;

            /*roll towards the rocket, raise arm*/
            case 3:
                PIDElevator(elevatorDeploy, elevatorPID);

                MOErioAuto.setHeading(robot.getHeadingDegrees() - approachHeading * LeftSide);
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower(0.6 * (1 + correction),
                        0.6 * (1 - correction));

                if(!withinArmTolerance){
                    raiseArm(armOut, armPID);
                }else{
                    autoStep++;
                }
                break;

            /*keep rolling, keep arm still and lower elevator*/
            case 4:
                if(robot.getElevatorEncoderCount()  <= elevatorFloor){
                    robot.driveElevator(0);
                }else{
                    robot.driveElevator(-0.3);
                }

                PIDArm(armOut, armPID);

                MOErioAuto.setHeading(robot.getHeadingDegrees() - approachHeading * LeftSide);
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower(0.6 * (1 + correction),
                        0.6 * (1 - correction));

                if(robot.getDistanceLeftInches() > 86){
                    autoStep++;
                    elevatorPID.resetError();
                }
                break;


            /*auto target w pixy and keeping elevator still*/
            case 5:

                PIDElevator(elevatorFloor,elevatorPID);

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

            /*spear out*/
            case 6:
                robot.spearOut();
                robot.setDrivePower(0.3,0.3);
                if(robot.lidar[0] < 500){
                    autoStep++;
                    startTime = System.currentTimeMillis();
                }
                break;

            /*drive forward for half a second*/
            case 7:
                robot.setDrivePower(0.2,0.2);
                if(System.currentTimeMillis() - 500 > startTime){
                    autoStep++;
                }
                break;


            //fingers in
            case 8:
                /* LFR */
                robot.spearHook();
                robot.stopDriving();
                autoStep++;
                robot.resetDriveEncoders();
                break;

            //back off for ????? in
            //spear in
            case 9:
                robot.spearHook();
                robot.spearIn();
                robot.setDrivePower(-0.2,-0.2);
                if(Math.abs(robot.getDistanceLeftInches()) > 24){
                    autoStep++;
                }
                break;

            case 10:
                robot.setDrivePower(0.5*LeftSide,-0.5*LeftSide);
                if(reachedHeadingHands(80,1*LeftSide)){
                    autoStep++;
                    MOErioAuto.resetError();
                    robot.resetDriveEncoders();
                }
                break;

            case 11:

                MOErioAuto.setHeading(robot.getHeadingDegrees()-90*LeftSide);
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower(0.3*(1 + correction),0.3*(1 - correction));

                if(robot.lidar[0] <= 580){
                    autoStep++;
                    MOErioAuto.resetError();
                }
                break;

            case 12:
                robot.setDrivePower(0.5*LeftSide,-0.5*LeftSide);
                if(reachedHeadingHands(170, LeftSide)){
                    autoStep++;
                    MOErioAuto.resetError();
                }
                break;

            case 13:
                MOErioAuto.setHeading(robot.getHeadingDegrees() - (Math.sin(robot.getHeadingDegrees() * Math.PI / 180)));
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower(0.3*(1 - correction),0.3*(1 + correction));
                if(robot.lidar[0] < 900){
                    autoStep++;
                }
                break;

            case 14:

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


            case 15:
                robot.stopDriving();
                break;
        }
    }


    //48 in, 60/50 deg, elevator up, arm up, elevator down, 86 inches, autoapproach

}
