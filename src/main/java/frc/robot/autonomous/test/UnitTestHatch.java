package frc.robot.autonomous.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.autonomous.GenericAuto;

public class UnitTestHatch extends GenericAuto {

    PIDModule MOEvaAuto = new PIDModule(0.06, 0.001, 0);
    double LouWizardry;
    double correction;
    long startTime = 0;

    @Override
    public void init() {
        autoStep = 0;
        robot.resetDriveEncoders();
        robot.resetYaw();
        MOEvaAuto.resetError();
        MOEvaAuto.setHeading(0);
        startTime = System.currentTimeMillis();
    }

    @Override
    public void printSmartDashboard() {

        SmartDashboard.putNumber("Error: ", MOEvaAuto.getInput());
        SmartDashboard.putNumber("Correction: ", MOEvaAuto.getCorrection());
        SmartDashboard.putNumber("kP: ", MOEvaAuto.pidController.getP());
        SmartDashboard.putNumber("kI: ", MOEvaAuto.pidController.getI());
        SmartDashboard.putNumber("kD: ", MOEvaAuto.pidController.getD());
        SmartDashboard.putNumber("Current Time: ", System.currentTimeMillis());
        SmartDashboard.putNumber("Abs Left", Math.abs(robot.getDistanceLeftInches()));
        SmartDashboard.putNumber("Abs Right", Math.abs(robot.getDistanceRightInches()));
    }

    public void run() {
        /*double leftDistance = robot.getDistanceLeftInches();
        double rightDistance = robot.getDistanceRightInches();*/
        final double TICKS_PER_INCH = 60;

        //LouWizardry = 66.5*robot.getHeadingDegrees()/180*3.1415972 - leftDistance;

        switch (autoStep) {
            case 0:
                if (robot.getElevatorEncoderCount()/TICKS_PER_INCH < 10.75) {
                    robot.driveElevator(0.5);
                    autoStep = 1;
                }
                break;
            case 1:
                robot.spearOut();
                robot.spearUnhook();
                autoStep = 2;
                break;
            case 2:
                robot.spearIn();
                robot.spearUnhook();
                autoStep = 3;
                break;
            case 3:
                if (robot.getElevatorEncoderCount()/TICKS_PER_INCH > 0) {
                    robot.driveElevator(0.5);
                    autoStep = 4;
                }
                break;
            case 4:
                robot.stopEverything();
                break;
        }
    }
}
