package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.MOErio;
import frc.robot.genericrobot.PIDModule;

public class MOErioCargoFrontAuto extends GenericAuto {

    PIDModule MOErioAuto = new PIDModule(0.06,0.001,0);
    //long startTime = 0;
    double z = 1.33;


    @Override
    public void init() {
        autoStep = 0;
        robot.resetDriveEncoder();
        robot.resetYaw();
        MOErioAuto.setHeading(0);
        MOErioAuto.resetError();
        MOErioAuto.setHeading(0);
        //startTime = System.currentTimeMillis();
    }

    @Override
    public void run() {
        double leftDistance = robot.getDistanceLeftInches();
        double rightDistance = robot.getDistanceRightInches();
        double louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) * z;

        SmartDashboard.putNumber("Error: ", MOErioAuto.getInput());
        SmartDashboard.putNumber("Correction: ", MOErioAuto.getCorrection());
        SmartDashboard.putNumber("kP: ", MOErioAuto.pidController.getP());
        SmartDashboard.putNumber("kI: ", MOErioAuto.pidController.getI());
        SmartDashboard.putNumber("kD: ", MOErioAuto.pidController.getD());
        SmartDashboard.putNumber("Current Time: ", System.currentTimeMillis());
        SmartDashboard.putNumber("The Magic: ", louWizardry);
        SmartDashboard.putNumber("Z: ",z);
        SmartDashboard.putNumber("Abs Left", Math.abs(robot.getDistanceLeftInches()));
        SmartDashboard.putNumber("Abs Right", Math.abs(robot.getDistanceRightInches()));

        switch (autoStep) {
            case 0:
                MOErioAuto.setHeading(louWizardry);
                double correction = MOErioAuto.getCorrection();

                //correction negative, left motor decrease, correction positive, left motor power increase
                robot.setDrivePower((0.5)*(1 + correction),(0.5)*(1 - correction));

                /* long now = System.currentTimeMillis();
                long endTime = startTime ;
                MOErioArc.setHeading(robot.getHeadingDegrees() - (dT * yawRate));*/

                if (Math.abs(robot.getDistanceLeftInches()) >= 48 ) {
                    autoStep++;
                    robot.resetDriveEncoder();
                }
                break;
            case 1:
        }
    }
}
