package frc.robot.autonomous.sandstorm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.autonomous.GenericAuto;

public class MASideAutoCargo extends GenericAuto {
    PIDModule MOErioAuto = new PIDModule(0.06, 0.001, 0);
    //PIDModuleLucy MOErioTurn = new PIDModuleLucy(2.5e-2, 1.75e-3, 0);
    double louWizardry = 0;

    double correction = 0;
    double zEffective;
    long startTime;

    PIDModule elevatorPID = new PIDModule(0.1, 0.00, 0);
    PIDModule armPID = new PIDModule(1.75e-2,3.0e-3,0);

    public double elevatorDeploy = 13.1;
    public double elevatorFloor = -30/*-3.13*/;
    public double armOut = 58;

    int topXVal;
    int numTimesNull = 0;
    long currentTime;
    int midCounter = 0;
    int pixyWait;

    double drivePower;

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

        withinArmTolerance = false;
        withinElevatorTolerance = false;
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
        double leftDistance = Math.abs(robot.getDistanceLeftInches());
        double rightDistance = Math.abs(robot.getDistanceRightInches());
        louWizardry = leftDistance - rightDistance * zEffective;

        switch (autoStep) {
            //wait for 0.1 sec
            case -2:
                MOErioAuto.resetError();
                robot.resetYaw();

                if (System.currentTimeMillis() >= startTime + 100) {
                    MOErioAuto.resetError();
                    robot.resetYaw();
                    autoStep++;
                }
                break;

            //drive off the HAB
            case -1:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();

                //correction negative, left motor decrease, correction positive, left motor power increase
                if (leftDistance<36)
                {
                    drivePower = robot.rampPower(0.3,0.8,0,36,leftDistance);
                }
                else
                {
                    drivePower = 0.8;
                }
                robot.setDrivePower((drivePower) * (1 + correction), (drivePower) * (1 - correction));

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

                if(!withinElevatorTolerance){
                    raiseElevator(elevatorDeploy,elevatorPID);
                }else{
                    PIDElevator(elevatorDeploy,elevatorPID);
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

                if(!withinElevatorTolerance){
                    raiseElevator(elevatorDeploy,elevatorPID);
                }else{
                    PIDElevator(elevatorDeploy,elevatorPID);
                }

                if (leftDistance >= 49) {
                    autoStep++;
                    MOErioAuto.resetError();
                    //MOErioTurn.resetError()
                }
                break;

            /*Right side- turn 90 degrees to the left*/
            /*Left side- turn 90 degrees to the right*/
            /*turning towards the hatch*/
            //raise elevator
            case 3:
                //MOErioTurn.setHeading(robot.getHeadingDegrees());
                PIDElevator(elevatorDeploy,elevatorPID);

                robot.setDrivePower(-0.5 * LeftSide, 0.5 * LeftSide);

                if (reachedHeadingHands(75, -1 * LeftSide)) {
                    //MOErioTurn.resetError();
                    robot.setDrivePower(0,0);
                    autoStep++;
                }
                break;

            /*turning cont'd*/
            case 4:
                autoStep++;
                break;

            //start raising arm (could we put this in an earlier step?)
            case 5:
                PIDElevator(elevatorDeploy,elevatorPID);

                if(!withinArmTolerance) {
                    raiseArm(armOut,armPID);
                } else {
                    autoStep++;
                }
                break;

            //pid control arm, lower elevator
            case 6:
                PIDArm(armOut,armPID);

                robot.driveElevator(-0.3);
                if(robot.getElevatorEncoderCount() <= elevatorFloor){
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                }
                break;

            /*roll towards the hatch*/
            case 7:
                PIDElevator(elevatorFloor + 3, elevatorPID);
                PIDArm(armOut, armPID);

                MOErioAuto.setHeading(robot.getHeadingDegrees() - -90*LeftSide);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower((0.4) * (1 + correction), (0.4) * (1 - correction));

                if(robot.lidar[0] <= 900){ // we may need to tweak this!
                    autoStep++;
                    robot.setDrivePower(0,0);
                    startTime = System.currentTimeMillis();
                }
                break;

            //pixy align
            case 8:
                PIDElevator(elevatorFloor+3,elevatorPID);
                PIDArm(armOut,armPID);

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

            //roll forward until lidar hits 850
            case 9:
                PIDElevator(elevatorFloor + 3,elevatorPID);
                PIDArm(armOut, armPID);

                MOErioAuto.setHeading(robot.getHeadingDegrees() + 90 * LeftSide);
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower(0.3 * (1 + correction), 0.3 * (1 - correction));

                if (robot.lidar[0] <= 850) { //750
                    autoStep++;
                    startTime = System.currentTimeMillis();
                }

                break;

            //roll out for 1.5 sec (do we need to roll out for this long?)
            case 10:
                PIDElevator(elevatorFloor + 3, elevatorPID);
                PIDArm(armOut,armPID);

                if (System.currentTimeMillis() - startTime > 1500) {
                    robot.rollOut(0);
                    robot.stopDriving();
                } else {
                    robot.rollOut(0.5);
                    robot.stopDriving();
                    autoStep++;
                }
                break;

            //fin.
            case 11:
                robot.stopDriving();
                break;

        }
    }
}
