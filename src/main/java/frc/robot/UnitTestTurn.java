package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class UnitTestTurn extends GenericAuto {
    PIDModule MOErioAuto = new PIDModule(0.06, 0.001, 0);
    long startTime = 0;

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
    public void printSmartDashboard() {

        SmartDashboard.putNumber("Error: ", MOErioAuto.getInput());
        SmartDashboard.putNumber("Correction: ", MOErioAuto.getCorrection());
        SmartDashboard.putNumber("kP: ", MOErioAuto.pidController.getP());
        SmartDashboard.putNumber("kI: ", MOErioAuto.pidController.getI());
        SmartDashboard.putNumber("kD: ", MOErioAuto.pidController.getD());
        SmartDashboard.putNumber("Current Time: ", System.currentTimeMillis());
        SmartDashboard.putNumber("Abs Left", Math.abs(robot.getDistanceLeftInches()));
        SmartDashboard.putNumber("Abs Right", Math.abs(robot.getDistanceRightInches()));
    }

    public void run() {
        double leftDistance = robot.getDistanceLeftInches();
        double rightDistance = robot.getDistanceRightInches();

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
                MOErioAuto.setHeading(robot.getHeadingDegrees());

                robot.setDrivePower(0.5, -0.5);

                if (robot.getHeadingDegrees() > 80) {
                    MOErioAuto.resetError();
                    autoStep++;
                }
                break;
            case 0:
                MOErioAuto.setHeading(robot.getHeadingDegrees()-90);
                double correction = MOErioAuto.getCorrection();
                robot.setDrivePower(0.5*correction,-0.5*correction);

                if (Math.abs(robot.getHeadingDegrees()-90) < 0.5)
                    ++autoStep;
                break;
            case 1:
                robot.stopDriving();
                break;
        }
    }
}
