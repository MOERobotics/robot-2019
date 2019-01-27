package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveStraightAuto extends GenericAuto {

    PIDModule straightPid = new PIDModule(0.06, 0.001,0.0);

    @Override
    public void init() {
        straightPid.resetError();
        straightPid.setHeading(0);
        autoStep = 0;
        robot.resetDriveEncoder();
        robot.resetYaw();

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
                straightPid.setHeading(robot.getHeadingDegrees());
                double correction = straightPid.getCorrection();
                //correction negative, left motor increase decrease. correction positive, left motor power increase.
                robot.setDrivePower(0.6*(1 + correction),0.6*(1 - correction));
                if(robot.getDistanceLeftInches() > 120) {
                    autoStep++;
                } else {
                    break;
                }
            case 1:
                robot.stopDriving();
        }
    }
}
