package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoFlyingFullRetraction extends GenericAuto {
    PIDModule elevatorPID = new PIDModule(/*0.1*/0.075, 0.00, 0);
    PIDModule armPID = new PIDModule(1.75e-2,3.0e-3,0);

    double elevatorCorrection;
    double armCorrection;
    double armPowerBias = 0;
    double elevatorDeploy = 37;
    double elevatorFloor = -28.6-3.13;
    //double elevatorBalance = -28;
    double elevatorBalance = 13;
    double elevatorFinale = 0;
    double armOut = /*40*/25;

    double hab3Height = 335;
    double retractHeight = 120;
    double fullRetractHeight = 5;
    double steadyPower;
    double startPitch;
    double startTime;

    boolean withinArmTolerance = false;

    int pulseCounter = 0;

    //pitch positive is up
    @Override
    public void init() {
        lastStep = 14;
        autoStep = 0;
        steadyPower = 0.3;
        habLevel = 3;
        withinArmTolerance = false;
    }

    @Override
    public void run() {
        SmartDashboard.putNumber("Climb autoStep: ", autoStep);

        switch(autoStep) {

            case -1:
                robot.spearHook();
                autoStep++;
                break;

            case 0:
                robot.climb(-1.0);

                if (robot.getElevatorEncoderCount() <= elevatorBalance - 5) {
                    robot.driveElevator(0.4);
                } else if (robot.getElevatorEncoderCount() >= elevatorBalance + 5) {
                    robot.driveElevator(-0.4);
                } else {
                    autoStep++;
                    elevatorPID.resetError();
                }
                break;

            case 1:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorBalance);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                //robot.footSpacerCylinder(true);
                //robot.driveArm(-0.1);
                robot.climb(-1.0);

                autoStep++;
                break;

            case 2:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorBalance);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                //robot.footSpacerCylinder(true);
                //robot.driveArm(-0.1);
                robot.climb(-1.0);

                if (Math.abs(robot.getClimberLEncoderCount()) >= hab3Height) {
                    robot.climb(0);
                    autoStep++;
                }
                break;

            case 3:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorBalance);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                //robot.footSpacerCylinder(true);
                robot.LinearSlider(DoubleSolenoid.Value.kReverse);
                startTime = System.currentTimeMillis();

                autoStep = 5;
                break;

            case 4:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorBalance);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                //robot.footSpacerCylinder(true);
                robot.driveArm(0.2);
                robot.setDrivePower(steadyPower,steadyPower);

                if (robot.getArmEncoderCount() >= armOut){
                    armPID.resetError();
                    startTime = System.currentTimeMillis();
                    autoStep++;
                }
                break;

            case 5:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorBalance);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                //armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                //armCorrection = armPID.getCorrection();
                //robot.driveArm(armPowerBias + armCorrection);

                //robot.footSpacerCylinder(true);
                robot.setDrivePower(steadyPower,steadyPower);

                if(System.currentTimeMillis() - startTime >= 2000){
                    robot.setDrivePower(0,0);
                    autoStep++;
                    startTime = System.currentTimeMillis();
                }
                break;

            case 6:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorBalance);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                //robot.footSpacerCylinder(true);
                robot.driveArm(0.2);
                //robot.setDrivePower(steadyPower,steadyPower);

                if ((robot.getArmEncoderCount() >= armOut) ||
                        (System.currentTimeMillis() - startTime > 3000)){
                    if (Math.abs(robot.getArmEncoderCount()-armOut) < 5) {
                        withinArmTolerance = true;
                        armPID.resetError();
                    }
                    robot.climb(1);
                    autoStep++;
                }
                break;

            case 7:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorFinale);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                if (withinArmTolerance) {
                    armPID.setHeading(robot.getArmEncoderCount() - armOut);
                    armCorrection = armPID.getCorrection();
                    robot.driveArm(armPowerBias + armCorrection);
                } else if (Math.abs(robot.getArmEncoderCount() - armOut) < 5) {
                    withinArmTolerance = true;
                    armPID.resetError();
                }

                //robot.footSpacerCylinder(true);
                //robot.climb(1.0);
                robot.setDrivePower(steadyPower,steadyPower);
                robot.climb(1);
                if (Math.abs(robot.getClimberLEncoderCount()) < 50 ||
                    Math.abs(robot.getClimberREncoderCount()) < 50) {
                    autoStep++;
                }
                break;

            case 8:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorFinale);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                armCorrection = armPID.getCorrection();
                robot.driveArm(armPowerBias + armCorrection);

                //robot.footSpacerCylinder(true);
                //robot.climb(1.0);
                //robot.setDrivePower(steadyPower,steadyPower);

                if(Math.abs(robot.getClimberLEncoderCount()) > fullRetractHeight + 15){
                    robot.climbLDown(1);
                } else if (Math.abs(robot.getClimberLEncoderCount()) < fullRetractHeight + 15
                        && Math.abs(robot.getClimberLEncoderCount()) > fullRetractHeight) {
                    robot.climbLDown(0.2);
                } else {
                    robot.climbLDown(0);
                }

                if(Math.abs(robot.getClimberREncoderCount()) > fullRetractHeight + 15) {
                    robot.climbRDown(1);
                } else if (Math.abs(robot.getClimberREncoderCount()) < fullRetractHeight + 15
                        && Math.abs(robot.getClimberREncoderCount()) > fullRetractHeight) {
                    robot.climbRDown(0.2);
                } else {
                    robot.climbRDown(0);
                }

                if(Math.abs(robot.getClimberLEncoderCount()) <= fullRetractHeight
                        && Math.abs(robot.getClimberREncoderCount()) <= fullRetractHeight){
                    autoStep++;
                    robot.climb(0);
                    robot.setDrivePower(steadyPower,steadyPower);
                    startTime = System.currentTimeMillis();
                    robot.driveArm(0);
                }
                break;

            case 9:
                if (System.currentTimeMillis() - startTime > 1500) {
                    robot.setDrivePower(0, 0);
                    autoStep++;
                } else {
                    robot.setDrivePower(steadyPower, steadyPower);
                }
                robot.driveArm(0);
                break;

            case 10:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorFinale);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                robot.driveArm(0);

                //armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                //armCorrection = armPID.getCorrection();
                //robot.driveArm(armPowerBias + armCorrection);

                robot.setDrivePower(0,0);
                robot.footSpacerCylinder(false);

                pulseCounter++;
                if(pulseCounter > 15){
                    autoStep++;
                    pulseCounter = 0;
                }
                break;

            case 11:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorFinale);
                robot.driveElevator(elevatorCorrection);

                //armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                //armCorrection = armPID.getCorrection();
                //robot.driveArm(armPowerBias + armCorrection);

                robot.driveArm(0);

                robot.setDrivePower(0,0);
                //robot.footSpacerCylinder(true);

                pulseCounter++;
                if(pulseCounter > 25){
                    autoStep++;
                    pulseCounter = 0;
                }
                break;

            case 12:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorFinale);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                robot.driveArm(0);
                //armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                //armCorrection = armPID.getCorrection();
                //robot.driveArm(armPowerBias + armCorrection);

                robot.setDrivePower(0,0);
                //robot.footSpacerCylinder(false);
                break;

            case 13:

                robot.driveArm(0);

                robot.resetDriveEncoders();
                //PIDTune to 0 distance on the ENCODERS
                robot.stopEverything();
                break;
        }
    }
}
