package frc.robot.autonomous.sandstorm;
//STILL NEEDS TESTING!

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.autonomous.GenericAuto;

public class MAShipFrontHatch1BetterReloadAuto extends GenericAuto  {
    PIDModule MOErioAuto = new PIDModule(0.06, 0.001, 0);
    //PIDModuleLucy MOErioTurn = new PIDModuleLucy(2.5e-2, 1.75e-3, 0);
    long startTime = 0;
    double louWizardry = 0;

    double correction = 0;
    double zEffective;

    int approachHeading = 8;

    double drivePower;

    PIDModule elevatorPID = new PIDModule(0.1, 0.00, 0);
    PIDModule armPID = new PIDModule(1.75e-2,3.0e-3,0);
    double elevatorDeploy = 13.1;
    double elevatorFloor = -30/*-3.13*/;
    double armOut = 20;

    int topXVal;
    int numTimesNull = 0;
    int pixyWait = 0; //frame counter for waiting between pixy adjustments
    long currentTime;
    int midCounter = 0;

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

        margin = 2;
        biggerMargin = 6;
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
                robot.setDrivePower((0.6) * (1 + correction), (0.6) * (1 - correction));

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

                if(!withinElevatorTolerance){
                    raiseElevator(elevatorDeploy,elevatorPID);
                }else{
                    autoStep++;
                }
                break;

            /*raise arm*/
            case 1:
                robot.stopDriving();
                PIDElevator(elevatorDeploy, elevatorPID);

                if(!withinArmTolerance){
                    raiseArm(armOut,armPID);
                }else{
                    autoStep++;
                }
                break;

            //keep em still
            case 2:
                PIDElevator(elevatorDeploy,elevatorPID);
                PIDArm(armOut,armPID);
                MOErioAuto.resetError();
                autoStep++;
                break;

            //drive toward cargo ship
            case 3:
                PIDElevator(elevatorDeploy,elevatorPID);
                PIDArm(armOut,armPID);
                MOErioAuto.setHeading(robot.getHeadingDegrees() - approachHeading * LeftSide);
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower(0.6 * (1 + correction),
                        0.6 * (1 - correction));

                if (robot.getDistanceLeftInches() > 55) { //64
                    startTime = System.currentTimeMillis();
                    autoStep++;
                }
                break;

            //auto targeting
            case 4:
                PIDElevator(elevatorDeploy, elevatorPID);
                PIDArm(armOut, armPID);

                currentTime = System.currentTimeMillis() - startTime;
                drivePower = a1 + (a2 * Math.exp( -((double) currentTime/lambda)));

                if (pixyWait < 5) {
                    pixyWait++;
                    break;
                }
                pixyWait = 0;
                if (robot.pixy.vec.length != 1) {
                    //Null counter, if not detecting pixy lines, don't move
                    numTimesNull++;
                    if (numTimesNull > 4) {
                        robot.setDrivePower(0, 0);
                    }
                } else {
                    numTimesNull = 0; //reset null exit counter
                    if (robot.pixy.vec.length != 0 && robot.pixy.vec[0] != null) {
                        //which point of vector is higher on screen? get that point's X val
                        //topXVal = robot.pixy.vec[0].getX1();
                        topXVal = (int) (0.8*robot.pixy.vec[0].getX1() + 0.2*robot.pixy.vec[0].getX0());
                        if (robot.pixy.vec[0].getY0() < robot.pixy.vec[0].getY1()) {
                            //topXVal = robot.pixy.vec[0].getX0();
                            topXVal = (int) (0.8*robot.pixy.vec[0].getX0() + 0.2*robot.pixy.vec[0].getX1());
                        }
                    }
                }

                if ( (currentTime < 2000) && (Math.abs(topXVal-midPoint) > margin)) {
                    if (topXVal - midPoint > margin) {
                        midCounter = 0;
                        robot.setDrivePower(drivePower, -drivePower);
                    } else if (topXVal - midPoint < -margin) {
                        midCounter = 0;
                        robot.setDrivePower(-drivePower, drivePower);
                    }
                } else {
                    ++midCounter;
                    robot.setDrivePower(0, 0);
                    if ((midCounter>5) || (currentTime >= 2000)) {
                        autoStep++;
                    }
                }
                break;

            //lower elevator
            case 5:
                PIDArm(armOut,armPID);

                robot.driveElevator(-0.3);
                if(robot.getElevatorEncoderCount()  <= elevatorFloor){
                    autoStep++;
                }
                break;

            //spear out, drive forward
            case 6:
                PIDArm(armOut, armPID);
                PIDElevator(elevatorFloor + 3, elevatorPID);
                robot.spearOut();
                robot.setDrivePower(0.3,0.3);
                if(robot.lidar[0] < 475){
                    autoStep++;
                    startTime = System.currentTimeMillis();
                }
                break;

            //drive forward one second
            case 7:
                PIDArm(armOut, armPID);
                PIDElevator(elevatorFloor + 3, elevatorPID);

                robot.setDrivePower(0.2,0.2);
                if(System.currentTimeMillis() - 1000 > startTime){
                    startTime = System.currentTimeMillis();
                    autoStep++;
                }
                break;

            //pause for 3 sec
            case 8:
                PIDArm(armOut, armPID);
                PIDElevator(elevatorFloor + 3, elevatorPID);

                robot.setDrivePower(0,0);
                if (System.currentTimeMillis() - 3000 > startTime) {
                    autoStep++;
                }
                break;

            //fingers in
            case 9:
                PIDArm(armOut, armPID);
                PIDElevator(elevatorFloor + 3, elevatorPID);

                robot.spearHook();
                robot.stopDriving();
                autoStep++;
                break;

            //roll back until lidar hits 500
            case 10:
                PIDArm(armOut, armPID);
                PIDElevator(elevatorFloor + 3, elevatorPID);

                robot.setDrivePower(-0.3,-0.3);
                if(robot.lidar[0] > 500){ //flipped the < – CHECK THIS!
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                }
                break;

            //fingers in, spear in. roll backwards
            case 11:
                PIDArm(armOut, armPID);
                PIDElevator(elevatorFloor + 3, elevatorPID);

                robot.spearIn();

                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower(-0.3 * (1 - correction),-0.3 * (1 + correction));
                if(leftDistance > 12){ //this distance needs to be confirmed
                    autoStep++;
                    robot.setDrivePower(0,0);
                }
                break;

            //turn around 90 degrees to the right [left]
            case 12:
                PIDArm(armOut, armPID);
                PIDElevator(elevatorFloor + 3, elevatorPID);

                robot.setDrivePower(0.3*LeftSide,-0.3*LeftSide);
                if(reachedHeadingHands(75,1*LeftSide)){
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                }
                break;

            //roll forward towards the wall until lidar hits 360
            case 13:
                PIDArm(armOut, armPID);
                PIDElevator(elevatorFloor + 3, elevatorPID);

                MOErioAuto.setHeading(robot.getHeadingDegrees() - 90*LeftSide);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(0.3*(1+correction), 0.3*(1-correction));
                if(robot.lidar[0] < 360){
                    robot.setDrivePower(0,0);
                    autoStep++;
                }
                break;

            //turn around another 90 degrees (so your yaw will be 180) to the right [left]
            case 14:
                PIDArm(armOut, armPID);
                PIDElevator(elevatorFloor + 3, elevatorPID);

                robot.setDrivePower(0.2*LeftSide, -0.2*LeftSide);
                if(reachedHeadingHands(170,1*LeftSide)){
                    autoStep++;
                    MOErioAuto.resetError();
                    robot.resetDriveEncoders();
                }
                break;

            //roll forwards + ramp power! about 108 inches (is this distance correct on MOEva as well? do we need to ramp?)
            case 15:
                PIDArm(armOut, armPID);
                PIDElevator(elevatorFloor + 3, elevatorPID);

                MOErioAuto.setHeading(180/Math.PI*(Math.sin(robot.getHeadingDegrees() * Math.PI / 180)));
                correction = MOErioAuto.getCorrection();
                if (leftDistance < 22) {
                    drivePower = robot.rampPower(0.3,0.8,0,22,leftDistance);
                } else if (leftDistance > 22 && leftDistance < 86){
                    drivePower = 0.8;
                } else if (leftDistance > 86) {
                    drivePower = robot.rampPower(0.8, 0.3, 86, 108, leftDistance);
                }
                robot.setDrivePower(drivePower*(1 - correction), drivePower*(1 + correction));

                if (robot.getDistanceLeftInches() > 108) {
                    autoStep++;
                }
                break;

            //continue until lidar is less than 800.
            case 16:
                PIDArm(armOut, armPID);
                PIDElevator(elevatorFloor + 3, elevatorPID);

                MOErioAuto.setHeading(180/Math.PI*(Math.sin(robot.getHeadingDegrees() * Math.PI / 180)));
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(0.3*(1 - correction), drivePower*(1 + correction));

                if(robot.lidar[0] < 800){
                    autoStep++;
                    robot.setDrivePower(0,0);
                    startTime = System.currentTimeMillis();
                }
                break;

            //straighten using pixy align
            case 17:
                PIDArm(armOut, armPID);
                PIDElevator(elevatorFloor + 3, elevatorPID);

                currentTime = System.currentTimeMillis() - startTime;
                drivePower = a1 + (a2 * Math.exp( -((double) currentTime/lambda)));

                if (pixyWait < 5) {
                    pixyWait++;
                    break;
                }
                pixyWait = 0;
                if (robot.pixy.vec.length != 1) {
                    //Null counter, if not detecting pixy lines, don't move
                    numTimesNull++;
                    if (numTimesNull > 4) {
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
                }

                if ( (currentTime < 2000) && (Math.abs(topXVal-midPoint) > margin)) {
                    if (topXVal - midPoint > margin) {
                        midCounter = 0;
                        robot.setDrivePower(drivePower, -drivePower);
                    } else if (topXVal - midPoint < -margin) {
                        midCounter = 0;
                        robot.setDrivePower(-drivePower, drivePower);
                    }
                } else {
                    ++midCounter;
                    robot.setDrivePower(0, 0);
                    if ((midCounter>5) || (currentTime >= 2000)) {
                        autoStep++;
                    }
                }
                break;

            //now roll forward until lidar hits 475 + spear out
            //YOU SHOULD BE IN FRONT OF THE LOADING STATION ACCEPTING A HATCH.
            case 18:
                PIDArm(armOut, armPID);
                PIDElevator(elevatorFloor + 3, elevatorPID);

                robot.spearOut();
                robot.setDrivePower(0.3,0.3);
                if(robot.lidar[0] < 475){//this needs to be confirmed
                    autoStep++;
                    startTime = System.currentTimeMillis();
                }
                break;

            //roll another second.
            case 19:
                PIDArm(armOut, armPID);
                PIDElevator(elevatorFloor + 3, elevatorPID);

                robot.setDrivePower(0.3,0.3);

                if(System.currentTimeMillis() - 1000 > startTime){
                    autoStep++;
                    robot.setDrivePower(0,0);
                }
                break;

            //fingers out
            case 20:
                PIDArm(armOut, armPID);
                PIDElevator(elevatorFloor + 3, elevatorPID);

                robot.spearUnhook();
                autoStep++;
                break;

            //stop. (drive backwards?)
            case 21:
                robot.stopDriving();
                break;

        }
    }

}
