package frc.robot.genericrobot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GenericAuto;
import frc.robot.genericrobot.MOErio;

public class MOErioTestZ extends GenericAuto {
    @Override
    public void init() {
        autoStep = 0;
        robot.resetDriveEncoder();
        robot.resetYaw();
    }

    @Override
    public void run() {
        double leftDistance = robot.getDistanceLeftInches();
        double rightDistance = robot.getDistanceRightInches();

        SmartDashboard.putNumber("'Left' Distance:", leftDistance);
        SmartDashboard.putNumber("'Right' Distance:", rightDistance);

        switch (autoStep) {
            case 0:
                robot.setDrivePower(0.5,0.3);

                if (leftDistance <= -96) {
                    autoStep++;
                }
                break;
            case 1:
                robot.stopDriving();
                break;
        }
    }
}
