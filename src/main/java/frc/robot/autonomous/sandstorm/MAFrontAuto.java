package frc.robot.autonomous.sandstorm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.PIDModuleLucy;
import frc.robot.autonomous.GenericAuto;

public class MAFrontAuto extends GenericAuto {
    //line this up on the far side of the HAB space, against level three.
    PIDModule MOErioAuto = new PIDModule(0.06, 0.001, 0);
    PIDModuleLucy MOErioTurn = new PIDModuleLucy(2.5e-2, 1.75e-3, 0);
    long startTime = 0;
    double louWizardry = 0;
    //int LeftSide = 1;
    int turncounter = 0;
    double moementumCorrection = 100;
    double zEffective;

    @Override
    public void init() {
        autoStep = -2;
        robot.resetDriveEncoders();
        robot.resetYaw();
        MOErioAuto.setHeading(0);
        MOErioAuto.resetError();
        MOErioAuto.setHeading(0);
        startTime = System.currentTimeMillis();
        z = 1.81;

        if (LeftSide==-1)
        {
            zEffective = 1/z;
        }
        else
        {
            zEffective = z;
        }
        levelTwo = false;
    }

    @Override
    public void printSmartDashboard() {
        /*SmartDashboard.putNumber("Error: ", MOErioAuto.getInput());
        SmartDashboard.putNumber("Correction: ", MOErioAuto.getCorrection());
        SmartDashboard.putNumber("kP: ", MOErioAuto.pidController.getP());
        SmartDashboard.putNumber("kI: ", MOErioAuto.pidController.getI());
        SmartDashboard.putNumber("kD: ", MOErioAuto.pidController.getD());
        SmartDashboard.putNumber("Current Time: ", System.currentTimeMillis());
        SmartDashboard.putNumber("The Magic: ", louWizardry);
        SmartDashboard.putNumber("Z: ",z);
        SmartDashboard.putNumber("Abs Left", Math.abs(robot.getDistanceLeftInches()));
        SmartDashboard.putNumber("Abs Right", Math.abs(robot.getDistanceRightInches()));
        SmartDashboard.putBoolean("Level Two", levelTwo);
        SmartDashboard.putNumber("Left Side", LeftSide);
        SmartDashboard.putNumber("Arc Lengths: ", 49);*/
    }


    @Override
    public void run() {
        double leftDistance = Math.abs(robot.getDistanceLeftInches());
        double rightDistance = Math.abs(robot.getDistanceRightInches());
        //LFR        louWizardry = leftDistance - rightDistance / zEffective;

        switch (autoStep) {
            case -2:
                MOErioAuto.resetError();
                robot.resetYaw();

                if (System.currentTimeMillis() >= startTime + 100) {
                    MOErioAuto.resetError();
                    robot.resetYaw();
                    autoStep = -1;
                }
                break;

            /*roll off the HAB*/
            case -1:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                double correction = MOErioAuto.getCorrection();

                //correction negative, left motor decrease, correction positive, left motor power increase
                robot.setDrivePower((0.8) * (1 + correction), (0.8) * (1 - correction));
                /*
                if (robot.getPitch() >= 3) {
                    autoStep++;
                    robot.resetDriveEncoders();
                }
                */
                if (levelTwo) {
                    if (leftDistance >= (46 - 8) * 2) {
                        autoStep++;
                        robot.resetDriveEncoders();
                    }
                } else if (leftDistance >= 46 - 8) {
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                }
                break;

            /*Right Side: we make a left turning arc*/
            /*Left Side: we make a right turning arc*/
            case 0:
                louWizardry = leftDistance - rightDistance / zEffective;

                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                setDrivePowerHands(0.1, 0.5, correction, LeftSide);

                if (getDistanceRightInchesHands(LeftSide) >= (51.0) /*x1*/) {
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                }
                break;

            /*Right Side: make a right turning arc*/
            /*Left Side: make a left turning arc*/
            /*you should be in front of the hatch*/
            case 1:
                louWizardry = leftDistance - rightDistance * zEffective;
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                setDrivePowerHands(0.5, 0.1, correction, LeftSide);

                if (getDistanceLeftInchesHands(LeftSide) >= 51.0) {
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                    MOErioTurn.resetError();
                }
                break;
            //59.4 in 121 t

            /*straightening out by turning in place*/
            case 2:
                MOErioTurn.setHeading(robot.getHeadingDegrees());
                correction = MOErioTurn.getCorrection();
                robot.setDrivePower(correction, -correction);

                if ((Math.abs(robot.getHeadingDegrees()) < 0.5) && (turncounter > 4)) {
                    ++autoStep;
                } else if (Math.abs(robot.getHeadingDegrees()) < 0.5) {
                    ++turncounter;
                } else {
                    turncounter = 0;
                }
                break;

            /*roll forward towards the hatch*/
            case 3:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower((0.3) * (1 + correction), (0.3) * (1 - correction));

                if (robot.lidar[0] <= 605 + moementumCorrection) {
                    autoStep++;
                }
                break;

            case 4:
                robot.stopDriving();
                break;
        }
    }
}
