package frc.robot.autonomous.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.GenericAuto;

public class PIDTesting extends GenericAuto {

    double leftPower = 0;
    double rightPower = 0;

    long startTime = 0;
    //long endTime = 0;

    public PIDTesting() {
        SmartDashboard.putNumber("leftPowerIn",0);
        SmartDashboard.putNumber("rightPowerIn", 0);
    }

    @Override
    public void init() {
        leftPower = SmartDashboard.getNumber("leftPowerIn",0);
        rightPower = SmartDashboard.getNumber("rightPowerIn", 0);
        autoStep = 1;
        startTime = System.currentTimeMillis();
    }

    @Override
    public void run() {
        switch (autoStep) {
            case 1:
                robot.setDrivePower(leftPower, rightPower);
                if (System.currentTimeMillis() > startTime + 3000) {
                    ++autoStep;
                }
                break;
            case 2:
                robot.stopDriving();
                break;
        }
    }
}
