package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveStraightAuto extends GenericAuto {

    PIDModule straightPid = new PIDModule(0.0, 0.00,0.0);
    long startTime = 0;

    @Override
    public void init() {
        straightPid.resetError();
        straightPid.setHeading(0);
        autoStep = 0;
        robot.resetDriveEncoder();
        robot.resetYaw();
        startTime = System.currentTimeMillis();

    }

    @Override
    public void run() {
        SmartDashboard.putNumber("error",straightPid.getInput());
        SmartDashboard.putNumber("correction", straightPid.getCorrection());
        SmartDashboard.putNumber("kP", straightPid.pidController.getP());
        SmartDashboard.putNumber("kI", straightPid.pidController.getI());
        SmartDashboard.putNumber("kD", straightPid.pidController.getD());
        switch (autoStep) {
            case 0:
                long now = System.currentTimeMillis();
                straightPid.setHeading(robot.getHeadingDegrees());
                double correction = straightPid.getCorrection();
                //correction negative, left motor decrease. correction positive, left motor power increase.
                robot.setDrivePower(0.3*(1 + correction),0.3*(1 - correction));
                if(robot.getDistanceLeftInches() >= 36) {
                    autoStep = 1;
                } else {
                    break;
                }
            case 1:
                robot.stopDriving();
        }
    }
}
