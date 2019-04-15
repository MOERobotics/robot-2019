package frc.robot.autonomous.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.autonomous.GenericAuto;

public class ArmMoves extends GenericAuto {
    PIDModule elevatorPID = new PIDModule(0.1, 0.00, 0);
    PIDModule armPID = new PIDModule(1.5e-2,3.0e-3,0);
    double elevatorOrigin = 18.4;
    double correction;
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

        SmartDashboard.putNumber("Error: ", elevatorPID.getInput());
        SmartDashboard.putNumber("Correction: ", correction);
        SmartDashboard.putNumber("kP: ", elevatorPID.pidController.getP());
        SmartDashboard.putNumber("kI: ", elevatorPID.pidController.getI());
        SmartDashboard.putNumber("kD: ", elevatorPID.pidController.getD());

    }

    @Override
    public void run() {

        switch(autoStep){
            case 0:
                robot.driveElevator(0.8);
                if(robot.getElevatorEncoderCount() >= 4.6){
                    autoStep++;
                    counter = 0;
                }
                break;

            case 1:
                elevatorPID.setHeading(robot.getElevatorEncoderCount() - elevatorOrigin );
                correction = elevatorPID.getCorrection();

                SmartDashboard.putNumber("Error: ", elevatorPID.getInput());
                SmartDashboard.putNumber("Correction: ", correction);
                SmartDashboard.putNumber("Current - Goal: ", robot.getElevatorEncoderCount() - elevatorOrigin);

                robot.driveElevator(correction);
                if(robot.getElevatorEncoderCount() <= elevatorOrigin + 1
                && robot.getElevatorEncoderCount() >= elevatorOrigin - 1){
                    counter++;
                }

                if(counter == 4) autoStep++;
                break;

            case 2:
            armPID.setHeading(robot.getArmEncoderCount() - armOrigin - armDifference);
            correction = armPID.getCorrection();

            SmartDashboard.putNumber("Error: ", armPID.getInput());
            SmartDashboard.putNumber("Correction: ", correction);
            SmartDashboard.putNumber("Current - Goal: ", robot.getArmEncoderCount() - armOrigin - armDifference);
            SmartDashboard.putNumber("kP: ", armPID.pidController.getP());
            SmartDashboard.putNumber("kI: ", armPID.pidController.getI());
            SmartDashboard.putNumber("kD: ", armPID.pidController.getD());

            robot.driveArm(armPowerBias + correction);

            break;

        }


    }
}
