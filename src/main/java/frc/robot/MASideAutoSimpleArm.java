package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MASideAutoSimpleArm extends GenericAuto {
    PIDModule MOErioAuto = new PIDModule(0.06, 0.001, 0);
    PIDModuleLucy MOErioTurn = new PIDModuleLucy(2.5e-2, 1.75e-3, 0);
    PIDModule elevatorPID = new PIDModule(0.1, 0.00, 0);
    PIDModule armPID = new PIDModule(1.75e-2,3.0e-3,0);

    double louWizardry = 0;
    double z = 1.33;
    double zEffective;
    double correction = 0;
    double elevatorCorrection;
    double armCorrection;

    double moementumCorrection = 100;
    double armPowerBias = 0;
    double elevatorDeploy = 13.1;
    double elevatorFloor = -28.6-3.13;
    double armOut = /*21.3*/ 21.0;

    boolean levelTwo = false;
    int turncounter = 0;
    long startTime = 0;

    double remainingDistance;


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
        lastStep = 10;
        LeftSide = 1;

        robot.resetDriveEncoders();
        robot.resetYaw();

        MOErioAuto.resetError();
        MOErioTurn.resetError();
        elevatorPID.resetError();
        armPID.resetError();

        startTime = System.currentTimeMillis();

        if (LeftSide == -1) {
            zEffective = 1 / z;
        } else {
            zEffective = z;
        }
    }

    @Override
    public void printSmartDashboard() {
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

                if (leftDistance >= 41 + 5) {
                    autoStep++;
                    MOErioAuto.resetError();
                    MOErioTurn.resetError();
                }
                break;

            /*Right side- turn 90 degrees to the left*/
            /*Left side- turn 90 degrees to the right*/
            /*turning towards the hatch*/
            case 3:
                MOErioTurn.setHeading(robot.getHeadingDegrees());

                robot.setDrivePower(-0.5 * LeftSide, 0.5 * LeftSide);

                if (reachedHeadingHands(80, -1 * LeftSide)) {
                    MOErioTurn.resetError();
                    autoStep++;
                }
                break;

            /*turning cont'd*/
            case 4:
                MOErioTurn.setHeading(robot.getHeadingDegrees() + 90 * LeftSide);
                correction = MOErioTurn.getCorrection();
                robot.setDrivePower(correction, -correction);

                if ((Math.abs(robot.getHeadingDegrees() + 90 * LeftSide) < 0.5) && (turncounter > 4)) {
                    robot.setDrivePower(0,0);
                    ++autoStep;
                } else if (Math.abs(robot.getHeadingDegrees() + 90 * LeftSide) < 0.5) {
                    ++turncounter;
                } else {
                    turncounter = 0;
                }
                break;

            case 5:
                robot.driveElevator(0.6);

                if(robot.getElevatorEncoderCount()  >= elevatorDeploy){
                    elevatorPID.resetError();
                    autoStep++;
                    remainingDistance = robot.lidar[0];//use lidar before lowering the arm/elevator to check the distance that's left
                    remainingDistance = remainingDistance / 25.4;//converting mm from lidar to inches
                }
                break;

            case 6:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorDeploy);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                robot.driveArm(0.2);
                if (robot.getArmEncoderCount()  >= armOut){
                    armPID.resetError();
                    autoStep++;
                }
                break;

            case 7:
                armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                armCorrection = armPID.getCorrection();
                robot.driveArm(armPowerBias + armCorrection);

                robot.driveElevator(-0.3);

                if(robot.getElevatorEncoderCount()  <= elevatorFloor){
                    MOErioAuto.resetError();
                    robot.resetDriveEncoders();
                    autoStep++;
                }
                break;

            case 8:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorFloor);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                armCorrection = armPID.getCorrection();
                robot.driveArm(armPowerBias + armCorrection);

                MOErioAuto.setHeading(robot.getHeadingDegrees() + 90 * LeftSide);
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower(0.3 * (1 + correction), 0.3 * (1 - correction));

                if(robot.getDistanceLeftInches() >= remainingDistance - 2){
                    autoStep++;
                }
                /*if (robot.lidar[0] <= 545 - 25.4) {
                    autoStep++;
                }*/

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

                robot.setDrivePower(0,0);
                break;
        }
    }
}
