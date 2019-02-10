package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DriveCurvedAuto extends GenericAuto {

    PIDModule straightPid = new PIDModule(0.06, 0.001,0.0);
    long startTime = 0;

    @Override
    public void init() {
        straightPid.resetError();
        straightPid.setHeading(0);
        autoStep = 0;
        robot.resetDriveEncoders();
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
                long dT = now - startTime;

                double yawRate = 5.0 / 1000;

                straightPid.setHeading(robot.getHeadingDegrees() - (dT*yawRate));

                //h = s - e
                //e = r - s
                //h = s - r + s

                double correction = straightPid.getCorrection();

                //correction negative, left motor increase decrease. correction positive, left motor power increase.
                robot.setDrivePower((0.2 + 0.082) * (1 + correction),(0.2 * (1 - correction)));

                if(dT > 10000) {
                    autoStep++;
                } else {
                    break;
                }
            case 1:
                robot.stopDriving();
        }
    }

}
