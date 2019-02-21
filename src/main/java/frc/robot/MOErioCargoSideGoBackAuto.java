package frc.robot;

/*will it work? ah life's mysteries*/

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.MOErio;
import frc.robot.PIDModule;

public class MOErioCargoSideGoBackAuto extends GenericAuto {

    PIDModule MOErioAuto = new PIDModule(0.06,0.001,0);
    long startTime = 0;
    double z = 1.33;
    double louWizardry = 0;
    boolean LeftSide = false;
    double correction = 0;
    //z = 1.67 for left = 0.5 right = 0.15 power
    //z = 1.33 for 0.5 and 0.3

    @Override
    public void printSmartDashboard() {
        double leftDistance = Math.abs(robot.getDistanceLeftInches());
        double rightDistance = Math.abs(robot.getDistanceRightInches());
        double louWizardry = rightDistance - leftDistance / z;

        SmartDashboard.putNumber("Error: ", MOErioAuto.getInput());
        SmartDashboard.putNumber("Correction: ", MOErioAuto.getCorrection());
        SmartDashboard.putNumber("kP: ", MOErioAuto.pidController.getP());
        SmartDashboard.putNumber("kI: ", MOErioAuto.pidController.getI());
        SmartDashboard.putNumber("kD: ", MOErioAuto.pidController.getD());
        SmartDashboard.putNumber("Current Time: ", System.currentTimeMillis());
        SmartDashboard.putNumber("The Magic: ", louWizardry);
        SmartDashboard.putNumber("Z: ", z);
        SmartDashboard.putNumber("Abs Left", Math.abs(robot.getDistanceLeftInches()));
        SmartDashboard.putNumber("Abs Right", Math.abs(robot.getDistanceRightInches()));

    }

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
    public void run() {
        double leftDistance = Math.abs(robot.getDistanceLeftInches());
        double rightDistance = Math.abs(robot.getDistanceRightInches());
        double louWizardry = rightDistance - leftDistance / z;

        switch(autoStep){
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
                MOErioAuto.setHeading((robot.getHeadingDegrees() - 90)*0.9);
                correction = MOErioAuto.getCorrection();
                //je ne sais pas
                double temp_correction = correction * -0.5;
                if (temp_correction > 0) temp_correction += 0.20;
                if (temp_correction < 0) temp_correction -= 0.20;
                robot.turnRightInplace(temp_correction);
                if (robot.getHeadingDegrees() >= 87 &&
                        robot.getHeadingDegrees() <= 93 &&
                        correction <  0.3 &&
                        correction > -0.3
                ) {
                    autoStep++;
                    MOErioAuto.resetError();
                    robot.resetDriveEncoders();
                    robot.resetYaw();
                }
                break;

            case 0:
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(-0.5*(1 - correction),-0.3*(1 + correction));

                if(rightDistance >= 96/z){
                    autoStep++;
                    MOErioAuto.resetError();
                    robot.resetDriveEncoders();
                }
                break;

            case 1:
                louWizardry = rightDistance - leftDistance * z;
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(-0.3*(1 - correction), -0.5*(1 + correction));

                if(rightDistance >= 96){
                    autoStep++;
                    MOErioAuto.resetError();
                    robot.resetDriveEncoders();
                }
                break;

            case 2:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(-0.3*(1 - correction), -0.3*(1 + correction));

                if(rightDistance >= 10){
                    autoStep++;
                }
                break;

            case 3:
                robot.stopDriving();
        }
    }
}

