package frc.robot.autonomous.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.autonomous.GenericAuto;


public class DriveCurvedAuto extends GenericAuto {

    //P = 0.06, I = 0.001, D = 0.0
    PIDModule straightPid = new PIDModule(0.06, 0.001,0.0);
    long startTime = 0;
    double louMagic = 0.082*1.5;

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
    public void printSmartDashboard() {
        SmartDashboard.putNumber("error",straightPid.getInput());
        SmartDashboard.putNumber("correction", straightPid.getCorrection());
        SmartDashboard.putNumber("kP", straightPid.pidController.getP());
        SmartDashboard.putNumber("kI", straightPid.pidController.getI());
        SmartDashboard.putNumber("kD", straightPid.pidController.getD());
    }

    @Override
    public void run() {
        switch (autoStep) {
            case 0:
                long now = System.currentTimeMillis();
                long dT = now - startTime;

                double yawRate = 5.0 / 1000;

                //sets error
                straightPid.setHeading(robot.getHeadingDegrees() - (dT*yawRate));

                //h = s - e
                //e = r - s
                //h = s - r + s
                double correction = straightPid.getCorrection();

                //correction negative, left motor decrease. correction positive, left motor power increase.
                robot.setDrivePower((0.2 + louMagic) * (1 + correction),(0.4 * (1 - correction)));

                if(dT > 4000) {
                    autoStep = 1;
                } else {
                    break;
                }
            /*case 1:
                now = System.currentTimeMillis();
                dT = now - startTime;

                yawRate = 5.0 / 1000;

                straightPID.setHeading(robot.getHeadingDegrees() - (10000-dT)*yawRate);
                correction = straightPID.getCorrection();
                robot.setDrivePower(0.3 * (1 + correction), (0.3 + louMagic) * (1 - correction));

                if(dT > 10000) {
                    autoStep++;
                } else {
                    break;
                }
                */
            case 1:
                robot.stopDriving();
        }
    }

}
