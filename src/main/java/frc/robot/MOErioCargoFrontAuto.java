package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.MOErio;

public class MOErioCargoFrontAuto extends GenericAuto {

    //1.67 for 0.5 0.15 power
    //1.33 for 0.5 0.3 power
    PIDModule MOErioAuto = new PIDModule(0.06, 0.001, 0);
    long startTime = 0;
    double z = 1.77;
    double louWizardry = 0;
    boolean LeftSide = false;
    boolean levelTwo = false;

    @Override
    public void init() {
        autoStep = -2;
        robot.resetDriveEncoders();
        robot.resetYaw();
        MOErioAuto.setHeading(0);
        MOErioAuto.resetError();
        MOErioAuto.setHeading(0);
        startTime = System.currentTimeMillis();
    }

    @Override
    public void printSmartDashboard() {

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
        SmartDashboard.putNumber("i am so angry right now", 6);
    }


    @Override
    public void run() {
        double leftDistance = robot.getDistanceLeftInches();
        double rightDistance = robot.getDistanceRightInches();
        louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) / z;


        switch (autoStep) {
            case -2:
                MOErioAuto.resetError();
                robot.resetYaw();

                if (System.currentTimeMillis() >= startTime + 100) {
                    MOErioAuto.resetError();
                    robot.resetYaw();
                    autoStep=-1;
                }
                break;
            case -1:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                double correction = MOErioAuto.getCorrection();

                //correction negative, left motor decrease, correction positive, left motor power increase
                robot.setDrivePower((0.5)*(1 + correction),(0.5)*(1 - correction));
                /*
                if (robot.getPitch() >= 3) {
                    autoStep++;
                    robot.resetDriveEncoders();
                }
                */
                if(levelTwo){
                    if(Math.abs(robot.getDistanceLeftInches()) >= 46*2){
                        if (LeftSide) {
                            autoStep = 4;
                        } else {
                            autoStep++;
                        }
                        robot.resetDriveEncoders();
                    }
                } else if(Math.abs(robot.getDistanceLeftInches()) >= 46) {
                    if (LeftSide) {
                        autoStep = 4;
                    } else {
                        autoStep++;
                    }
                    robot.resetDriveEncoders();
                }
                break;

            case 0:
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                //correction negative, left motor decrease, correction positive, left motor power increase
                robot.setDrivePower((0.15)*(1 + correction),(0.5)*(1 - correction));

                if (Math.abs(robot.getDistanceLeftInches()) >= (53+5) / z /*x1*/) {
                    autoStep=1;
                    robot.resetDriveEncoders();
                }
                break;

            case 1:
                louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) * z;
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower((0.5)*(1 + correction), (0.15)*(1 - correction));

                if (Math.abs(robot.getDistanceLeftInches()) >= 53+5){
                    autoStep=2;
                    robot.resetDriveEncoders();
                }
                break;
                //59.4 in 121 t
            case 2:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower((0.3)*(1 + correction),(0.3)*(1 - correction));

                if (Math.abs(robot.getDistanceLeftInches()) >= 6) {
                    autoStep++;
                }
                break;
            case 3:
                robot.stopDriving();
                break;
            case 4:
                louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) * z;
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower((0.5)*(1 + correction), (0.15)*(1 - correction));

                if (Math.abs(robot.getDistanceLeftInches()) >= 53){
                    autoStep++;
                    robot.resetDriveEncoders();
                }
                break;
            case 5:
                louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) / z;
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower((0.15)*(1 + correction), (0.5)*(1 - correction));

                if (Math.abs(robot.getDistanceLeftInches()) >= 53/z){
                    autoStep = 2;
                    robot.resetDriveEncoders();
                }
                break;
        }
    }
}
