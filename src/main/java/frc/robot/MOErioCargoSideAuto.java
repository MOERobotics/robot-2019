package frc.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MOErioCargoSideAuto extends GenericAuto {

    PIDModule MOErioAuto = new PIDModule(0.06,0.001,0);
    long startTime = 0;
    double z = 1.33;
    double louWizardry = 0;
    boolean habLeftSide = false;
    boolean levelTwo = false;
    /*LEFT SIDE CASES START AT 4*/

    @Override
    public void init() {
        autoStep = -2;
        robot.resetDriveEncoders();
        robot.resetYaw();
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
        SmartDashboard.putNumber("Abs Left",  Math.abs(robot.getDistanceLeftInches()));
        SmartDashboard.putNumber("Abs Right", Math.abs(robot.getDistanceRightInches()));
    }

    @Override
    public void run() {
        double leftDistance = robot.getDistanceLeftInches();
        double rightDistance = robot.getDistanceRightInches();
        louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) * z;

        switch (autoStep) {
            case -2:
                MOErioAuto.resetError();
                robot.resetYaw();

                if (System.currentTimeMillis() >= startTime + 100) {
                    MOErioAuto.resetError();
                    robot.resetYaw();
                    autoStep++;
                }
                break;
            case -1:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                double correction = MOErioAuto.getCorrection();

                //correction negative, left motor decrease, correction positive, left motor power increase
                robot.setDrivePower((0.5)*(1 + correction),(0.5)*(1 - correction));

                if(levelTwo){
                    if(robot.getDistanceLeftInches() >= 48*2) {
                        if(habLeftSide){
                            autoStep = 4;
                        } else{
                            autoStep=0;
                        }
                        robot.resetDriveEncoders();
                    }
                } else {
                    if(robot.getDistanceLeftInches() >= 48) {
                        if(habLeftSide){
                            autoStep = 4;
                        } else{
                            autoStep=0;
                        }
                        robot.resetDriveEncoders();
                    }
                }

                break;
            case 0:
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower((0.5)*(1 + correction),(0.3)*(1 - correction));


                if (Math.abs(robot.getDistanceLeftInches()) >= 72 /*x1*/) {
                    autoStep=1;
                    robot.resetDriveEncoders();
                }
                break;
            case 1:
                louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) / z;
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower((0.3)*(1 + correction),(0.5)*(1 - correction));

                if (Math.abs(robot.getDistanceLeftInches()) >= 43 / z /*x2*/) {
                    autoStep=2;
                    robot.resetDriveEncoders();
                }
                break;
            case 2:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower((0.4)*(1 + correction),(0.4)*(1 - correction));

                if (Math.abs(robot.getDistanceLeftInches()) >= 41) {
                    autoStep=3;
                    robot.resetYaw();
                }
                break;
            case 3:
                robot.turnLeftInplace(0.4);
                if (robot.getHeadingDegrees() <= -85) {
                    autoStep=8;
                }
                break;
            /*LEFT SIDE*/
            case 4:
                louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) / z;
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower((0.3)*(1 + correction),(0.5)*(1 - correction));

                if (Math.abs(robot.getDistanceRightInches()) >= 72 /*x2*/) {
                    autoStep=5;
                    robot.resetDriveEncoders();
                }
                break;

            case 5:
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower((0.5)*(1 + correction),(0.3)*(1 - correction));

                if (Math.abs(robot.getDistanceRightInches()) >= 43/z /*x1*/) {
                    autoStep=6;
                    robot.resetDriveEncoders();
                }
                break;

            case 6:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower((0.4)*(1 + correction),(0.4)*(1 - correction));

                if (Math.abs(robot.getDistanceLeftInches()) >= 41) {
                    autoStep=7;
                    robot.resetYaw();
                }
                break;

            case 7:
                robot.turnRightInplace(0.4);
                if (robot.getHeadingDegrees() <= 85) {
                    autoStep=8;
                }
                break;
            case 8:
                robot.stopDriving();
        }
    }
}
