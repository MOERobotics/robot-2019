package frc.robot.autonomous.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.autonomous.GenericAuto;

public class DeployArm extends GenericAuto {
    PIDModule elevatorPID = new PIDModule(0.1, 0.00, 0);
    PIDModule armPID = new PIDModule(1.75e-2,3.0e-3,0);
    double elevatorCorrection;
    double armCorrection;
    double armPowerBias = 0;
    double elevatorDeploy = 13.1;
    double elevatorFloor = -31;
    double armOut = 19;

    @Override
    public void init() {
        autoStep = 0;
        elevatorPID.resetError();
        armPID.resetError();
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
                if(robot.getElevatorEncoderCount()  >= elevatorDeploy){
                    autoStep++;
                    elevatorPID.resetError();
                }
                break;

            case 1:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorDeploy);
                elevatorCorrection = elevatorPID.getCorrection();

                robot.driveElevator(elevatorCorrection);

                robot.driveArm(0.4);
                if (robot.getArmEncoderCount()  >= armOut){
                    armPID.resetError();
                    autoStep++;
                }
                break;

            case 2:
                armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                armCorrection = armPID.getCorrection();

                robot.driveArm(armPowerBias + armCorrection);

                robot.driveElevator(-0.3);
                if(robot.getElevatorEncoderCount()  <= elevatorFloor){
                    autoStep++;
                }
                break;

            case 3:
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorFloor);
                elevatorCorrection = elevatorPID.getCorrection();

                robot.driveElevator(elevatorCorrection);

                armPID.setHeading(robot.getArmEncoderCount()  - armOut);
                armCorrection = armPID.getCorrection();

                robot.driveArm(armPowerBias + armCorrection);
                break;

        }

        //might be setting a little high (unbag 3/20)


    }
}
