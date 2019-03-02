package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TestZ extends GenericAuto {
    int setDistance = 96;

    @Override
    public void init() {
        autoStep = 0;
        robot.resetDriveEncoders();
        robot.resetYaw();
        SmartDashboard.putNumber("set distance", setDistance);

    }

    @Override
    public void run() {
        switch (autoStep) {
            case 0:

                robot.setDrivePower(0.5,0.15);

                if(Math.abs(robot.getDistanceLeftInches()) > setDistance) {
                    autoStep = 1;
                } else {
                    break;
                }

            case 1:
                robot.stopDriving();
        }
    }

}
