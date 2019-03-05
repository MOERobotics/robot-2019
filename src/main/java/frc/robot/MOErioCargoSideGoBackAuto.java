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
    //int LeftSide = 1;
    int turncounter = 0;
    double correction = 0;
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

    //always pass in positive degrees
    public boolean reachedHeadingHands(int degrees, int Handedness){
        if(Handedness==1) {
            if(robot.getHeadingDegrees() >= degrees){
                return true;
            }
        } else if(Handedness==-1) {
            if(robot.getHeadingDegrees() <= degrees * Handedness){
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
                MOErioAuto.setHeading(robot.getHeadingDegrees());

                robot.setDrivePower(0.5*LeftSide, -0.5*LeftSide);

                if (reachedHeadingHands(80,LeftSide)) {
                    MOErioAuto.resetError();
                    autoStep++;
                }
                break;
            case 0:
                MOErioAuto.setHeading(robot.getHeadingDegrees()-90*LeftSide);
                double correction = MOErioAuto.getCorrection();
                robot.setDrivePower(correction*LeftSide,-correction*LeftSide);

                if ( (Math.abs(robot.getHeadingDegrees()-90*LeftSide) < 0.5) && (turncounter >4) ) {
                    ++autoStep;
                }
                else if (Math.abs(robot.getHeadingDegrees()-90*LeftSide) < 0.5)
                {
                    ++turncounter;
                }
                else {
                    turncounter = 0;
                }
                break;

            case 1:
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(-0.5*(1 - correction),-0.3*(1 + correction));

                if(rightDistance >= 96/z){
                    autoStep=4;
                    MOErioAuto.resetError();
                    robot.resetDriveEncoders();
                }
                break;

            case 2:
                louWizardry = getDistanceRightInchesHands(LeftSide) - getDistanceLeftInchesHands(LeftSide) * z;
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                setDrivePowerHands(-0.3,-0.5,correction,LeftSide);

                if(getDistanceRightInchesHands(LeftSide) >= 96){
                    autoStep++;
                    MOErioAuto.resetError();
                    robot.resetDriveEncoders();
                }
                break;

            case 3:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(-0.3*(1 - correction), -0.3*(1 + correction));

                if(rightDistance >= 10){
                    autoStep++;
                }
                break;

            case 4:
                robot.stopDriving();
        }
    }
}

