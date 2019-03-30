package frc.robot.autonomous.sandstorm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.autonomous.GenericAuto;

public class MOErioCargoFrontGoBackAuto extends GenericAuto {

    PIDModule MOErioAuto = new PIDModule(0.06, 0.001, 0);
    long startTime = 0;
    double z = 1.81;
    //int LeftSide = 1;
    //-1 is left, 1 is right
    int turncounter = 0;
    //z = 1.67 for left = 0.5 right = 0.15 power
    //z = 1.33 for 0.5 and 0.3

    public void setDrivePowerHands(double left, double right, double correction, int Handedness) {
        if (!(Handedness==-1))
        {
            robot.setDrivePower(left*(1 + correction),right*(1 - correction));
        }
        else
        {
            robot.setDrivePower(right * (1 + correction), left * (1 - correction));
        }
    }

    public double getDistanceLeftInchesHands(int Handedness) {
        if (!(Handedness==-1)) {
            return (Math.abs(robot.getDistanceLeftInches()));
        } else {
            return (Math.abs(robot.getDistanceRightInches()));
        }
    }
    public double getDistanceRightInchesHands(int Handedness) {
        if (!(Handedness==-1)) {
            return (Math.abs(robot.getDistanceRightInches()));
        } else {
            return (Math.abs(robot.getDistanceLeftInches()));
        }
    }

    //always pass in negative degrees
    public boolean reachedHeadingHands(int degrees, int Handedness){
        if(Handedness==1) {
            if(robot.getHeadingDegrees() <= degrees){
                return true;
            }
        } else if(Handedness==-1) {
            if(robot.getHeadingDegrees() >= degrees * Handedness){
                return true;
            }
        } else {
            return false;
        }
        return false;
    }

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
        double louWizardry = getDistanceRightInchesHands(LeftSide) - getDistanceLeftInchesHands(LeftSide) / z;

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
                MOErioAuto.setHeading(louWizardry);
                double correction = MOErioAuto.getCorrection();

                setDrivePowerHands(-0.5,-0.1,correction,LeftSide);
                if(getDistanceRightInchesHands(LeftSide) >= 69.6/z){
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.setHeading(robot.getHeadingDegrees() - 45*LeftSide);
                    MOErioAuto.resetError();
                }
                break;
            case 0:
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(-0.5*(1-correction), -0.5*(1+correction));

                if(getDistanceRightInchesHands(LeftSide) >= 43+20){
                    autoStep++;
                    robot.resetDriveEncoders();

                }
                break;
            case 1:
                louWizardry = getDistanceRightInchesHands(LeftSide) - getDistanceLeftInchesHands(LeftSide) * z;
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                setDrivePowerHands(-0.1,-0.5,correction,LeftSide);

                if(getDistanceRightInchesHands(LeftSide) >= 69.6 /*it was 59.7 before(Lou's math)*/){
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.setHeading(robot.getHeadingDegrees());
                    MOErioAuto.resetError();
                }
                break;
            case 2:
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(-0.3*(1-correction), -0.3*(1+correction));

                if(Math.abs(robot.getDistanceRightInches()) >= 13.5 ){
                    autoStep++;
                }
                break;
            case 3:
                robot.stopDriving();
                break;






            /*Retired cases*/
            case 4:
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(-0.1*(1-correction),-0.5*(1+correction));
                if(Math.abs(robot.getDistanceLeftInches()) >= 69.6){
                    autoStep++;
                    robot.resetDriveEncoders();
                }
                break;
            case 5:
                MOErioAuto.setHeading(robot.getHeadingDegrees() + 60);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(-0.5*(1-correction), -0.5*(1+correction));

                if(Math.abs(robot.getDistanceLeftInches()) >= 75){
                    autoStep++;
                    robot.resetDriveEncoders();

                }
                break;
            case 6:
                louWizardry = rightDistance - leftDistance * z;
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(-0.5*(1-correction), -0.1*(1+correction));

                if(Math.abs(robot.getDistanceLeftInches()) >= 69.6 / z /*it was 59.7 before(Lou's math)*/){
                    autoStep = 2;
                    robot.resetDriveEncoders();
                }
                break;
        }
    }
}