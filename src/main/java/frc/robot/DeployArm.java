package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DeployArm extends GenericAuto{
    PIDModule elevatorPID = new PIDModule(0.1, 0.00, 0);
    PIDModule armPID = new PIDModule(1.5e-2,3.0e-3,0);
    double elevatorOrigin = 18.4;
    double elevatorCorrection;
    double armCorrection;
    double armDifference = 20.3;
    double armOrigin = -13.9;
    int counter = 0;
    double armPowerBias = 0;

    @Override
    public void init() {
        autoStep = 2;
        elevatorPID.resetError();

    }

    @Override
    public void printSmartDashboard(){

        SmartDashboard.putNumber("Elevator Error: ", elevatorPID.getInput());
        SmartDashboard.putNumber("Arm Error: ", armPID.getInput());
        SmartDashboard.putNumber("Elevator Correction: ", elevatorCorrection);
        SmartDashboard.putNumber("Arm Correction: ", armCorrection);
        SmartDashboard.putNumber("Elevator kP: ", elevatorPID.pidController.getP());
        SmartDashboard.putNumber("Elevator kI: ", elevatorPID.pidController.getI());
        SmartDashboard.putNumber("Elevator kD: ", elevatorPID.pidController.getD());
        SmartDashboard.putNumber("Arm kP: ", armPID.pidController.getP());
        SmartDashboard.putNumber("Arm kI: ", armPID.pidController.getI());
        SmartDashboard.putNumber("Arm kD: ", armPID.pidController.getD());

    }

    @Override
    public void run() {

        switch(autoStep){
            case 0:
                robot.driveElevator(0.8);
                if(robot.getElevatorEncoderCount() - elevatorOrigin >= 13.1){
                    autoStep++;
                    elevatorPID.resetError();
                }
                break;

            case 1:
                elevatorPID.setHeading(robot.getElevatorEncoderCount() - elevatorOrigin - 13.1);
                elevatorCorrection = elevatorPID.getCorrection();

                robot.driveElevator(elevatorCorrection);

                robot.driveArm(0.4);
                if (robot.getArmEncoderCount() - armOrigin >= 20.3){
                    armPID.resetError();
                    autoStep++;
                }
                break;

            case 2:
                armPID.setHeading(robot.getArmEncoderCount() - armOrigin - 20.3);
                armCorrection = armPID.getCorrection();

                robot.driveArm(armPowerBias + armCorrection);

                robot.driveElevator(-0.4);
                if(robot.getElevatorEncoderCount() - elevatorOrigin <= -28.6){
                    autoStep++;
                }
                break;

            case 3:
                elevatorPID.setHeading(robot.getElevatorEncoderCount() - elevatorOrigin - 28.6);
                elevatorCorrection = elevatorPID.getCorrection();

                robot.driveElevator(elevatorCorrection);

                armPID.setHeading(robot.getArmEncoderCount() - armOrigin - 20.3);
                armCorrection = armPID.getCorrection();

                robot.driveArm(armPowerBias + armCorrection);
                break;

        }


    }
}
