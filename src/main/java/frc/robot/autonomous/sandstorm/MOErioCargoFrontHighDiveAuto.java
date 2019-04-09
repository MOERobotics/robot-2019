package frc.robot.autonomous.sandstorm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.autonomous.GenericAuto;

public class MOErioCargoFrontHighDiveAuto extends GenericAuto {

    PIDModule MOErioAuto = new PIDModule(0.06, 0.001, 0);
    long startTime = 0;
    double z = 1.81;
    double louWizardry =0;

    @Override
    public void init() {
        autoStep = -3;
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
    }


    @Override
    public void run() {
        double leftDistance = robot.getDistanceLeftInches();
        double rightDistance = robot.getDistanceRightInches();
        louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) / z;


        switch (autoStep) {
            case -3:
                MOErioAuto.resetError();
                robot.resetYaw();

                if (System.currentTimeMillis() >= startTime + 100) {
                    MOErioAuto.resetError();
                    robot.resetYaw();
                    autoStep++;
                }
                break;
            case -2:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                double correction = MOErioAuto.getCorrection();

                robot.setDrivePower((0.5)*(1 + correction),(0.5)*(1 - correction));

                if(Math.abs(robot.getDistanceLeftInches()) >= 46) {
                    autoStep++;
                    robot.resetDriveEncoders();
                }
                break;
            case -1:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();

                //correction negative, left motor decrease, correction positive, left motor power increase
                robot.setDrivePower((0.5)*(1 + correction),(0.5)*(1 - correction));
                /*
                if (robot.getPitch() >= 3) {
                    autoStep++;
                    robot.resetDriveEncoder();
                }
                */
                if(Math.abs(robot.getDistanceLeftInches()) >= 46) {
                    autoStep=0;
                    robot.resetDriveEncoders();
                }
                break;

            case 0:
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                //correction negative, left motor decrease, correction positive, left motor power increase
                robot.setDrivePower((0.1)*(1 + correction),(0.5)*(1 - correction));

                if (Math.abs(robot.getDistanceLeftInches()) >= 52 / z /*x1*/) {
                    autoStep=1;
                    robot.resetDriveEncoders();
                }
                break;

            case 1:
                louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) * z;
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower((0.5)*(1 + correction), (0.1)*(1 - correction));

                if (Math.abs(robot.getDistanceLeftInches()) >= 52){
                    autoStep=2;
                    robot.resetDriveEncoders();
                }
                break;
            case 2:
                louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) * z;
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower((0.3)*(1 + correction), (0.3)*(1 - correction));

                if (Math.abs(robot.getDistanceLeftInches()) >= 5) {
                    autoStep++;
                    robot.resetDriveEncoders();
                }
            case 3:
                robot.stopDriving();
        }
    }
}
