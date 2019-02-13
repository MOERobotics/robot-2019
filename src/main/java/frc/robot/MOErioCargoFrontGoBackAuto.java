package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MOErioCargoFrontGoBackAuto extends GenericAuto {

    PIDModule MOErioAuto = new PIDModule(0.06, 0.001, 0);
    long startTime = 0;
    double z = 1.67;
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
        double leftDistance = robot.getDistanceLeftInches();
        double rightDistance = robot.getDistanceRightInches();
        double louWizardry = Math.abs(rightDistance) - Math.abs(leftDistance) / z;

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

                robot.setDrivePower(-0.5*(1-correction),-0.15*(1+correction));
                if(Math.abs(robot.getDistanceRightInches()) >= 59.7/z){
                    autoStep++;
                }
                break;
            case 0:
                MOErioAuto.setHeading(robot.getHeadingDegrees() - 45);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(-0.5*(1-correction), -0.5*(1+correction));

                if(Math.abs(robot.getDistanceRightInches()) >= 43){
                    autoStep++;
                }
            case 1:
                robot.stopDriving();
                break;
        }
    }
}