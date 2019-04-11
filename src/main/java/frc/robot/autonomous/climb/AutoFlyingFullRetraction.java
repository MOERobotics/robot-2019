package frc.robot.autonomous.climb;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.autonomous.GenericAuto;

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
    double pitchMin = Double.MAX_VALUE;

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
        autoStep = -1;
        steadyPower = 0.3;
        habLevel = 3;
        withinArmTolerance = false;
    }

    @Override
    public void run() {
        SmartDashboard.putNumber("Climb autoStep: ", autoStep);
        if (robot.getPitchDegrees() < pitchMin) {
            SmartDashboard.putNumber("Minimum Pitch", robot.getPitchDegrees());
            pitchMin = robot.getPitchDegrees();
        }

        switch(autoStep) {

            //Retracting the spear
            case -1:
                robot.spearHook();
                robot.driveRoller(0);
                autoStep++;
                break;

            //Start climb, move elevator up to balance height
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

            //Start PID controlling the elevator, continue climbing
            case 1:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorBalance);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                robot.climb(-1.0);

                autoStep++;
                break;

            //Continue climbing until bot is above Hab 3, continue PID controlling the elevator
            case 2:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorBalance);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                robot.climb(-1.0);

                if (Math.abs(robot.getClimberLEncoderCount()) >= hab3Height) {
                    robot.climb(0);
                    autoStep++;
                }
                break;

            //Push bot forward with linear sliders, continue PID controlling the elevator
            case 3:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorBalance);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                //robot.footSpacerCylinder(true);
                robot.LinearSlider(DoubleSolenoid.Value.kReverse);
                startTime = System.currentTimeMillis();

                autoStep = 5;
                break;

            //Move arm out, start driving forward, continue PID controlling elevator
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

            //Drive forward for 2 seconds, continue PID controlling elevator
            case 5:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorBalance);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                robot.setDrivePower(steadyPower,steadyPower);

                if(System.currentTimeMillis() - startTime >= 2000){
                    robot.setDrivePower(0,0);
                    autoStep++;
                    startTime = System.currentTimeMillis();
                }
                break;

            //Move arm out OR continue through the climb after 3 seconds, continue PID controlling elevator
            case 6:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorBalance);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

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

            //Start PID controlling elevator at finale height, drive forward,
            //continue moving arm up if not done already, start retracting feet
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

                robot.setDrivePower(steadyPower,steadyPower);
                robot.climb(1);
                if (Math.abs(robot.getClimberLEncoderCount()) < 50 ||
                    Math.abs(robot.getClimberREncoderCount()) < 50) {
                    autoStep++;
                }
                break;

            //Retract feet until at full retract height, continue PID controlling elevator, arm
            case 8:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorFinale);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                armCorrection = armPID.getCorrection();
                robot.driveArm(armPowerBias + armCorrection);

                //robot.setDrivePower(0,0);

                if(Math.abs(robot.getClimberLEncoderCount()) > fullRetractHeight + 15){
                    robot.climbLDown(1);
                } else if (Math.abs(robot.getClimberLEncoderCount()) < fullRetractHeight + 15
                        && Math.abs(robot.getClimberLEncoderCount()) > fullRetractHeight) {
                    robot.climbLDown(0.3);
                } else {
                    robot.climbLDown(0);
                }

                if(Math.abs(robot.getClimberREncoderCount()) > fullRetractHeight + 15) {
                    robot.climbRDown(1);
                } else if (Math.abs(robot.getClimberREncoderCount()) < fullRetractHeight + 15
                        && Math.abs(robot.getClimberREncoderCount()) > fullRetractHeight) {
                    robot.climbRDown(0.3);
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

            //Drive forward for 1.5 seconds, stop controlling the arm
            case 9:
                if (System.currentTimeMillis() - startTime > 1500) {
                    robot.setDrivePower(0, 0);
                    autoStep++;
                } else {
                    robot.setDrivePower(steadyPower, steadyPower);
                }
                robot.driveArm(0);
                break;

            //Wait for 15 pulses, continue PID controlling elevator
            case 10:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorFinale);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);
                robot.driveArm(0);

                robot.setDrivePower(0,0);
                robot.footSpacerCylinder(false);

                pulseCounter++;
                if(pulseCounter > 15){
                    autoStep++;
                    pulseCounter = 0;
                }
                break;

            //Wait for 25 pulses, continue PID controlling elevator
            case 11:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorFinale);
                robot.driveElevator(elevatorCorrection);

                robot.driveArm(0);

                robot.setDrivePower(0,0);

                pulseCounter++;
                if(pulseCounter > 25){
                    autoStep++;
                    pulseCounter = 0;
                }
                break;

            //Continue PID controlling elevator - end of climb
            case 12:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorFinale);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                robot.driveArm(0);

                robot.setDrivePower(0,0);
                break;

            //Unreachable step - Stop everything
            case 13:

                robot.driveArm(0);

                robot.resetDriveEncoders();
                //PIDTune to 0 distance on the ENCODERS
                robot.stopEverything();
                break;
        }
    }
}
