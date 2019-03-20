package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoFlying extends GenericAuto {
    PIDModule elevatorPID = new PIDModule(0.1, 0.00, 0);
    PIDModule armPID = new PIDModule(1.75e-2,3.0e-3,0);

    double elevatorCorrection;
    double armCorrection;
    double armPowerBias = 0;
    double elevatorDeploy = 37;
    double elevatorFloor = -28.6-3.13;
    double elevatorBalance = -28;
    double armOut = 47;

    double hab3Height = 350;
    double steadyPower;
    double startPitch;
    double startTime;

    int pulseCounter = 0;

    //pitch positive is up
    @Override
    public void init() {
        lastStep = 14;
        autoStep = 0;
        steadyPower = 0.3;
        habLevel = 3;
    }

    @Override
    public void run() {

        switch(autoStep) {
             /* assuming Alex has rested both arm and elevator on the HAB*/

            case 0:
                robot.driveElevator(-0.4);
                robot.climb(-1.0);

                if(robot.getElevatorEncoderCount() <= elevatorBalance){
                    autoStep++;
                    elevatorPID.resetError();
                }
                break;

            case 1:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorBalance);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                robot.footSpacerCylinder(true);
                robot.driveArm(-0.1);
                robot.climb(-1.0);

                autoStep++;
                break;

            case 2:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorBalance);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                robot.footSpacerCylinder(true);
                robot.driveArm(-0.1);
                robot.climb(-1.0);

                if(Math.abs(robot.getClimberLEncoderCount()) >= hab3Height) {
                    autoStep++;
                    robot.climb(0);
                }
                break;

            case 3:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorBalance);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                robot.footSpacerCylinder(true);
                robot.LinearSlider(DoubleSolenoid.Value.kReverse);

                autoStep++;
                break;

            case 4:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorBalance);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                robot.footSpacerCylinder(true);
                robot.driveArm(0.2);
                robot.setDrivePower(steadyPower,steadyPower);

                if (robot.getArmEncoderCount()  >= armOut){
                    armPID.resetError();
                    startTime = System.currentTimeMillis();
                    autoStep++;
                }
                break;

            case 5:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorBalance);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                armCorrection = armPID.getCorrection();
                robot.driveArm(armPowerBias + armCorrection);

                robot.footSpacerCylinder(true);
                robot.setDrivePower(steadyPower,steadyPower);

                if(System.currentTimeMillis() - startTime >= 2000){
                    autoStep++;
                }
                break;

            case 6:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorBalance);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                armCorrection = armPID.getCorrection();
                robot.driveArm(armPowerBias + armCorrection);

                robot.footSpacerCylinder(true);
                robot.climb(1.0);
                robot.setDrivePower(steadyPower,steadyPower);

                if(Math.abs(robot.getClimberLEncoderCount()) <= 120){
                    autoStep++;
                    robot.climb(0);
                    robot.setDrivePower(0,0);
                }
                break;

            case 7:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorBalance);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                armCorrection = armPID.getCorrection();
                robot.driveArm(armPowerBias + armCorrection);

                robot.setDrivePower(0,0);
                robot.footSpacerCylinder(false);

                pulseCounter++;
                if(pulseCounter > 15){
                    autoStep++;
                    pulseCounter = 0;
                }
                break;

            case 8:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorBalance);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                armCorrection = armPID.getCorrection();
                robot.driveArm(armPowerBias + armCorrection);

                robot.setDrivePower(0,0);
                robot.footSpacerCylinder(true);

                pulseCounter++;
                if(pulseCounter > 25){
                    autoStep++;
                    pulseCounter = 0;
                }
                break;

            case 9:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorBalance);
                elevatorCorrection = elevatorPID.getCorrection();
                robot.driveElevator(elevatorCorrection);

                armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                armCorrection = armPID.getCorrection();
                robot.driveArm(armPowerBias + armCorrection);

                robot.setDrivePower(0,0);
                robot.footSpacerCylinder(false);
                break;

            case 13:

                robot.resetDriveEncoders();
                //PIDTune to 0 distance on the ENCODERS
                robot.stopEverything();
                break;
        }
    }
}
