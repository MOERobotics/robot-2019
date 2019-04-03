package frc.robot.autonomous.sandstorm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.PIDModuleLucy;
import frc.robot.autonomous.GenericAuto;

public class MASideAutoCargo extends GenericAuto {
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
    boolean levelTwo = false;

    PIDModule elevatorPID = new PIDModule(0.1, 0.00, 0);
    PIDModule armPID = new PIDModule(1.75e-2,3.0e-3,0);
    double elevatorCorrection;
    double armCorrection;
    double armPowerBias = 0;
    double elevatorDeploy = 13.1;
    double elevatorFloor = -28.6/*-3.13*/;
    double armOut = 53;

    double orientationTolerance = 0.5;

    int midPoint = 40;
    int margin = 2; //set margin of error where it wont move at all (prevents jittering)

    int numTimesNull = 0;

    double turnPower = 0.45;

    public void setDrivePowerHands(double left, double right, double correction, int Handedness) {
        if (!(Handedness == -1)) {
            robot.setDrivePower(left * (1 + correction), right * (1 - correction));
        } else {
            robot.setDrivePower(right * (1 + correction), left * (1 - correction));
        }
    }

    public double getDistanceLeftInchesHands(int Handedness) {
        if (!(Handedness == -1)) {
            return (Math.abs(robot.getDistanceLeftInches()));
        } else {
            return (Math.abs(robot.getDistanceRightInches()));
        }
    }

    public double getDistanceRightInchesHands(int Handedness) {
        if (!(Handedness == -1)) {
            return (Math.abs(robot.getDistanceRightInches()));
        } else {
            return (Math.abs(robot.getDistanceLeftInches()));
        }
    }

    //pass in degrees and direction
    //1 = to the right
    //-1 = to the left
    public boolean reachedHeadingHands(int degrees, int Handedness) {
        if (Handedness == 1) {
            if (robot.getHeadingDegrees() >= degrees) {
                return true;
            }
        } else if (Handedness == -1) {
            if (robot.getHeadingDegrees() <= degrees * Handedness) {
                return true;
            }
        } else {
            return false;
        }
        return false;
    }

    //case 7, bonus begins and normal auto ends

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
        double leftDistance = Math.abs(robot.getDistanceLeftInches());
        double rightDistance = Math.abs(robot.getDistanceRightInches());
        louWizardry = leftDistance - rightDistance * zEffective;

        switch (autoStep) {
            case -2:
                MOErioAuto.resetError();
                robot.resetYaw();

                if (System.currentTimeMillis() >= startTime + 100) {
                    MOErioAuto.resetError();
                    robot.resetYaw();
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


                if(robot.getElevatorEncoderCount() < elevatorDeploy){
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

                if(robot.getElevatorEncoderCount() < elevatorDeploy){
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
                //MOErioTurn.setHeading(robot.getHeadingDegrees());

                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorDeploy);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                robot.setDrivePower(-0.5 * LeftSide, 0.5 * LeftSide);

                if (reachedHeadingHands(80, -1 * LeftSide)) {
                    //MOErioTurn.resetError();
                    robot.setDrivePower(0,0);
                    autoStep++;
                }
                break;

            /*turning cont'd*/

            case 4:
                autoStep++;
                break;

            case 5:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorDeploy);
                elevatorCorrection = elevatorPID.getCorrection();

                robot.driveElevator(elevatorCorrection);

                robot.driveArm(0.2);
                if (robot.getArmEncoderCount()  >= armOut){
                    armPID.resetError();
                    autoStep++;
                }
                break;

            case 6:
                armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                armCorrection = armPID.getCorrection();

                robot.driveArm(armPowerBias + armCorrection);

                robot.driveElevator(-0.3);
                if(robot.getElevatorEncoderCount()  <= elevatorFloor){
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                }
                break;

            /*roll towards the hatch*/

            case 7:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorFloor);
                elevatorCorrection = elevatorPID.getCorrection();

                robot.driveElevator(elevatorCorrection);

                armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                armCorrection = armPID.getCorrection();

                robot.driveArm(armPowerBias + armCorrection);

                MOErioAuto.setHeading(robot.getHeadingDegrees() - -90*LeftSide);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower((0.4) * (1 + correction), (0.4) * (1 - correction));

                if(leftDistance >= 36){ // we may need to tweak this!
                    autoStep++;
                    robot.setDrivePower(0,0);
                }
                break;

            case 8:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorFloor);
                elevatorCorrection = elevatorPID.getCorrection();

                robot.driveElevator(elevatorCorrection);

                armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                armCorrection = armPID.getCorrection();

                robot.driveArm(armPowerBias + armCorrection);

                if (robot.pixy.vec.length != 1) {
                    numTimesNull++;
                    if (numTimesNull > 4) {
                        //autoStep = 2;
                        //System.out.println("Null PixyCam Vectors, next auto activated");
                        robot.setDrivePower(0,0);
                    }
                } else {
                    //vec = vectors[0]; //set vec
                    numTimesNull = 0; //reset null exit counter
                    //System.out.println("Pixy Vector: "+vec.toString());
                    //System.out.println("test 1");

/*
                    if (robot.pixy.vec.length == 1) {
                        System.out.println(robot.pixy.vec[0].getX0());
                    }
*/

                    if (robot.pixy.vec.length != 0 && robot.pixy.vec[0] != null) {
                        //System.out.println("test 2");
                        //which point of vector is higher on screen? get that point's X val
                        int topXVal = robot.pixy.vec[0].getX1();
                        if (robot.pixy.vec[0].getY0() > robot.pixy.vec[0].getY1()) {
                            topXVal = robot.pixy.vec[0].getX0();
                        }

                        if (topXVal > midPoint + margin) {
                            robot.setDrivePower(turnPower,-turnPower);
                        } else if (topXVal < midPoint - margin) {
                            robot.setDrivePower(-turnPower,turnPower);
                        } else {
                            autoStep++;
                            robot.setDrivePower(0,0);
                        }
                    }
                }
                break;

            case 9:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorFloor);
                elevatorCorrection = elevatorPID.getCorrection();

                robot.driveElevator(elevatorCorrection);

                armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                armCorrection = armPID.getCorrection();

                robot.driveArm(armPowerBias + armCorrection);

                MOErioAuto.setHeading(robot.getHeadingDegrees() + 90 * LeftSide);
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower(0.3 * (1 + correction), 0.3 * (1 - correction));

                if (robot.lidar[2] <= 750) {
                    autoStep++;
                    startTime = System.currentTimeMillis();
                }

                break;

            case 10:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorFloor);
                elevatorCorrection = elevatorPID.getCorrection();

                robot.driveElevator(elevatorCorrection);

                armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                armCorrection = armPID.getCorrection();

                robot.driveArm(armPowerBias + armCorrection);

                if (System.currentTimeMillis() - startTime > 500) {
                    robot.rollOut(0);
                    robot.stopDriving();
                } else {
                    robot.rollOut(0.5);
                }
                break;

        }
    }
}
