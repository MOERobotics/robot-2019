package frc.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.MOErio;
import frc.robot.PIDModule;

public class MOErioCargoFrontGoBackAuto extends GenericAuto {

    PIDModule MOErioAuto = new PIDModule(0.06, 0.001, 0);
    long startTime = 0;
    double z = 1.81;
    //z = 1.67 for left = 0.5 right = 0.15 power
    //z = 1.33 for 0.5 and 0.3



    @Override
    public void init() {
        autoStep = -2;
        robot.resetDriveEncoders();
        robot.resetYaw();
        MOErioAuto.resetError();
        MOErioAuto.setHeading(0);
        startTime = System.currentTimeMillis();
    }

    @Override
    public void run() {
        double leftDistance = Math.abs(robot.getDistanceLeftInches());
        double rightDistance = Math.abs(robot.getDistanceRightInches());
        double louWizardry = rightDistance - leftDistance / z;

        SmartDashboard.putNumber("Error: ", MOErioAuto.getInput());
        SmartDashboard.putNumber("Correction: ", MOErioAuto.getCorrection());
        SmartDashboard.putNumber("kP: ", MOErioAuto.pidController.getP());
        SmartDashboard.putNumber("kI: ", MOErioAuto.pidController.getI());
        SmartDashboard.putNumber("kD: ", MOErioAuto.pidController.getD());
        SmartDashboard.putNumber("Current Time: ", System.currentTimeMillis());
        SmartDashboard.putNumber("The Magic: ", louWizardry);
        SmartDashboard.putNumber("Z: ", z);
        SmartDashboard.putNumber("Abs Left", Math.abs(robot.getDistanceLeftInches()));
        SmartDashboard.putNumber("Abs Right", Math.abs(robot.getDistanceRightInches()));

        switch (autoStep) {
            case -2:
                MOErioAuto.resetError();
                robot.resetYaw();

                if (System.currentTimeMillis() >= startTime + 100) {
                    MOErioAuto.resetError();
                    robot.resetYaw();
                    autoStep = -1;
                }
                break;
            case -1:
                MOErioAuto.setHeading(louWizardry);
                double correction = MOErioAuto.getCorrection();

                robot.setDrivePower(-0.5*(1-correction),-0.1*(1+correction));
                if(Math.abs(robot.getDistanceRightInches()) >= 69.6/z){
                    autoStep++;
                    robot.resetDriveEncoders();
                }
                break;
            case 0:
                MOErioAuto.setHeading(robot.getHeadingDegrees() + 60);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(-0.5*(1-correction), -0.5*(1+correction));

                if(Math.abs(robot.getDistanceRightInches()) >= 75){
                    autoStep++;
                    robot.resetDriveEncoders();

                }
                break;
            case 1:
                louWizardry = rightDistance - leftDistance * z;
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(-0.1*(1-correction), -0.5*(1+correction));

                if(Math.abs(robot.getDistanceRightInches()) >= 69.6 /*it was 59.7 before(Lou's math)*/){
                    autoStep++;
                    robot.resetDriveEncoders();
                }
                break;
            case 2:
                louWizardry = rightDistance - leftDistance * z;
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(-0.3*(1-correction), -0.3*(1+correction));

                if(Math.abs(robot.getDistanceRightInches()) >= 13.5 /*it was 59.7 before(Lou's math)*/){
                    autoStep++;
                }
                break;
            case 3:
                robot.stopDriving();
                break;
        }
    }
}