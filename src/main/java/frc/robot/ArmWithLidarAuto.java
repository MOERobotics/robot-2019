package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmWithLidarAuto extends GenericAuto {

    PIDModule straightPID = new PIDModule(0.06, 0.001,0.0);
    PIDModule elevatorPID = new PIDModule(0.1, 0.00, 0);
    PIDModule armPID = new PIDModule(1.75e-2,3.0e-3,0);

    double louWizardry = 0;
    double correction = 0;
    double elevatorCorrection;
    double armCorrection;

    double moementumCorrection = 100;
    double elevatorDeploy = 13.1;
    double elevatorFloor = -28.6-3.13;
    double armOut = /*21.3*/ 21.0;
    int counter;

    double remainingDistance;
    double distanceAway = 16; //the distance we want to be from the ship in inches

    @Override
    public void init() {
        robot.shiftLow();

        straightPID.resetError();
        elevatorPID.resetError();
        armPID.resetError();

        autoStep = -1;
        lastStep = 7;

        robot.resetDriveEncoders();
        robot.resetYaw();

        counter = 0;

        remainingDistance = robot.lidar[0] / 25.4; //the distance we are from the ship converted to inches
    }

    @Override
    public void printSmartDashboard() {
        SmartDashboard.putNumber("Distance From Ship At Start: ", remainingDistance);
        SmartDashboard.putNumber("Distance We Should Be From Ship At End: ", distanceAway);
        SmartDashboard.putNumber("Distance We Should Roll: ", remainingDistance - distanceAway);
        SmartDashboard.putNumber("Counter: ", counter);
    }

    @Override
    public void run() {
        switch (autoStep) {
            case -1:
                if (Math.abs(robot.getElevatorEncoderCount() - elevatorDeploy) < 5) {
                    elevatorPID.resetError();
                    autoStep++;
                } else if (robot.getElevatorEncoderCount() < elevatorDeploy)
                {
                    robot.driveElevator(0.5);
                }
                else
                {
                    robot.driveElevator(-0.5);
                }
                break;

            case 0:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorDeploy);
                elevatorCorrection = elevatorPID.getCorrection();

                robot.driveElevator(elevatorCorrection);

                if (Math.abs(robot.getArmEncoderCount() - armOut) < 5){
                    armPID.resetError();
                    autoStep++;
                }
                else if (robot.getArmEncoderCount() < armOut)
                {
                    robot.driveArm(0.4);
                }
                else
                {
                    robot.driveArm(-0.4);
                }
                break;

            case 1:
                armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                armCorrection = armPID.getCorrection();

                robot.driveArm(armCorrection);

                if (Math.abs(robot.getElevatorEncoderCount() - elevatorFloor) < 5) {
                    elevatorPID.resetError();
                    straightPID.resetError();
                    autoStep++;
                } else if (robot.getElevatorEncoderCount() < elevatorFloor)
                {
                    robot.driveElevator(0.5);
                }
                else
                {
                    robot.driveElevator(-0.5);
                }
                break;

            case 2:
                armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                armCorrection = armPID.getCorrection();

                robot.driveArm(armCorrection);

                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorFloor);
                elevatorCorrection = elevatorPID.getCorrection();

                robot.driveElevator(elevatorCorrection);

                robot.spearUnhook();
                straightPID.resetError();

                autoStep++;
                break;

            case 3:
                armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                armCorrection = armPID.getCorrection();

                robot.driveArm(armCorrection);

                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorFloor);
                elevatorCorrection = elevatorPID.getCorrection();

                robot.driveElevator(elevatorCorrection);

                straightPID.setHeading(robot.getHeadingDegrees());
                correction = straightPID.getCorrection();

                robot.setDrivePower(0.3*(1 + correction),0.3*(1 - correction));

                if(Math.abs(robot.getDistanceLeftInches()) >= remainingDistance - distanceAway) {
                    autoStep++;
                }
                break;

            case 4:
                armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                armCorrection = armPID.getCorrection();

                robot.driveArm(armCorrection);

                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorFloor);
                elevatorCorrection = elevatorPID.getCorrection();

                robot.driveElevator(elevatorCorrection);

                straightPID.setHeading(robot.getHeadingDegrees());
                correction = straightPID.getCorrection();

                robot.setDrivePower(0.3*(1 + correction),0.3*(1 - correction));

                counter++;

                if(counter > 10){
                    autoStep++;
                }
                break;


            case 5:
                armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                armCorrection = armPID.getCorrection();

                robot.driveArm(armCorrection);

                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorFloor);
                elevatorCorrection = elevatorPID.getCorrection();

                robot.driveElevator(elevatorCorrection);

                robot.stopDriving();

                robot.spearOut();

                autoStep++;
                break;

            case 6:
                armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                armCorrection = armPID.getCorrection();

                robot.driveArm(armCorrection);

                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorFloor);
                elevatorCorrection = elevatorPID.getCorrection();

                robot.driveElevator(elevatorCorrection);

                robot.stopDriving();

                robot.spearHook();

                break;
        }
    }
}
