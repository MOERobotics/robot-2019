package frc.robot;

import frc.robot.genericrobot.PIDModule;

public class DriveStraightAuto extends GenericAuto {

    PIDModule straightPid = new PIDModule(0.1,0.008,0);
    long startTime = 0;

    @Override
    public void init() {
        autoStep = 0;
        robot.resetDriveEncoder();
        robot.resetYaw();
        straightPid.setHeading(0);
        straightPid.resetError();
        startTime = System.currentTimeMillis();
    }

    @Override
    public void run() {
        switch (autoStep) {
            case 0:
                long now = System.currentTimeMillis();
                long dT = now - startTime;

                double yawRate = 5.0 / 1000.0;
                double louMagic = 0.082;

                straightPid.setHeading(robot.getHeadingDegrees() - (dT * yawRate));

                double correction = straightPid.getCorrection();

                robot.setDrivePower(
                        (louMagic + 0.2) * (1 + correction),
                        0.2 * (1 - correction)
                );
                if (dT > 10000) {
                    autoStep = 1;
                } else {
                    break;
                }
            case 1:
                robot.stopDriving();
                break;
        }
        }
}
