package frc.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.MOErio;
import frc.robot.PIDModule;

public class MOErioCargoFrontGoBackAuto extends GenericAuto {

    PIDModule MOErioAuto = new PIDModule(0.06, 0.001, 0);
    long startTime = 0;
    double z = 1.81;
    boolean LeftSide = false;
    double correction =0 ;
    //z = 1.67 for left = 0.5 right = 0.15 power
    //z = 1.33 for 0.5 and 0.3



    @Override
    public void init() {
        autoStep = -1;
        robot.resetDriveEncoders();
        robot.resetYaw();
        MOErioAuto.resetError();
        MOErioAuto.setHeading(0);
        startTime = System.currentTimeMillis();
    }

    public void printSmartDashboard() {

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

        SmartDashboard.putString("AAAA","AAAA");
    }
    @Override
    public void run() {
        double leftDistance = Math.abs(robot.getDistanceLeftInches());
        double rightDistance = Math.abs(robot.getDistanceRightInches());
        double louWizardry = rightDistance - leftDistance / z;

        switch (autoStep) {
            case -3:
                MOErioAuto.resetError();
                robot.resetYaw();

                if (System.currentTimeMillis() >= startTime + 100) {
                    MOErioAuto.resetError();
                    robot.resetYaw();
                    autoStep=-1;
                }
                    break;
            case -2:
               robot.turnLeftInplace(0.5);

               if (robot.getHeadingDegrees() >= 179) {
                   if (LeftSide) {
                       autoStep = 4;
                   } else {
                       autoStep++;
                   }
               }
               robot.resetDriveEncoders();
               robot.resetYaw();
                break;
            case -1:
                MOErioAuto.setHeading(louWizardry);
                double correction = MOErioAuto.getCorrection();

                robot.setDrivePower(-0.5*(1-correction),-0.1*(1+correction));
                if(Math.abs(robot.getDistanceRightInches()) >= 69.6/z){
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.setHeading(robot.getHeadingDegrees() - 45);
                    MOErioAuto.resetError();
                }
                break;
            case 0:
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(-0.5*(1-correction), -0.5*(1+correction));

                if(Math.abs(robot.getDistanceLeftInches()) >= 86){
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
                    MOErioAuto.resetError();
                    MOErioAuto.setHeading(robot.getHeadingDegrees());
                }
                break;
            case 2:
                louWizardry = rightDistance - leftDistance * z;
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(-0.3*(1-correction), -0.3*(1+correction));

                if(Math.abs(robot.getDistanceRightInches()) >= 13.5 /*it was 59.7 before(Lou's math)*/){
                    autoStep++;
                }
                break;
            case 3:
                robot.stopDriving();
                break;
            case 4:
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(0.1*(1-correction),0.5*(1+correction));
                if(Math.abs(robot.getDistanceLeftInches()) >= 69.6){
                    autoStep++;
                    robot.resetDriveEncoders();

                    MOErioAuto.setHeading(robot.getHeadingDegrees() + 45);
                    MOErioAuto.resetError();
                }
                break;
            case 5:
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(0.5*(1-correction), 0.5*(1+correction));

                if(Math.abs(robot.getDistanceLeftInches()) >= 75){
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.setHeading(robot.getHeadingDegrees() + 60);
                    MOErioAuto.resetError();
                }
                break;
            case 6:
                louWizardry = rightDistance - leftDistance * z;
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(0.5*(1-correction), 0.1*(1+correction));

                if(Math.abs(robot.getDistanceLeftInches()) >= 69.6 / z /*it was 59.7 before(Lou's math)*/){
                    autoStep = 2;
                    robot.resetDriveEncoders();
                }
                break;
        }
    }
}