package frc.robot.autonomous.sandstorm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.PIDModuleLucy;
import frc.robot.autonomous.GenericAuto;

public class MASideAuto extends GenericAuto {
    PIDModule MOErioAuto = new PIDModule(0.06, 0.001, 0);
    PIDModuleLucy MOErioTurn = new PIDModuleLucy(2.5e-2, 1.75e-3, 0);
    long startTime = 0;
    double louWizardry = 0;
    //int LeftSide = 1;
    //-1 is left, 1 is right
    int turncounter = 0;
    double correction = 0;
    double moementumCorrection = 100;
    double zEffective;

    //case 7, bonus begins and normal auto ends

    @Override
    public void init() {
        autoStep = -2;
        robot.resetDriveEncoders();
        robot.resetYaw();
        MOErioAuto.resetError();
        MOErioAuto.setHeading(0);
        MOErioTurn.resetError();
        startTime = System.currentTimeMillis();

        if (LeftSide == -1) {
            zEffective = 1 / z;
        } else {
            zEffective = z;
        }
    }

    @Override
    public void printSmartDashboard() {
        correction = MOErioAuto.getCorrection();
        SmartDashboard.putNumber("Error: ", MOErioAuto.getInput());
        SmartDashboard.putNumber("Correction: ", correction);
        SmartDashboard.putNumber("kP: ", MOErioAuto.pidController.getP());
        SmartDashboard.putNumber("kI: ", MOErioAuto.pidController.getI());
        SmartDashboard.putNumber("kD: ", MOErioAuto.pidController.getD());
        SmartDashboard.putNumber("Current Time: ", System.currentTimeMillis());
        SmartDashboard.putNumber("The Magic: ", louWizardry);
        SmartDashboard.putNumber("Z: ", z);
        SmartDashboard.putNumber("Abs Left", Math.abs(robot.getDistanceLeftInches()));
        SmartDashboard.putNumber("Abs Right", Math.abs(robot.getDistanceRightInches()));
        SmartDashboard.putNumber("Left Side", LeftSide);
        SmartDashboard.putBoolean("Level Two", levelTwo);
    }

    @Override
    public void run() {
        double leftDistance = Math.abs(robot.getDistanceLeftInches());
        double rightDistance = Math.abs(robot.getDistanceRightInches());
        louWizardry = leftDistance - rightDistance * zEffective;

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

            /*drive off the HAB*/
            case -1:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();

                //correction negative, left motor decrease, correction positive, left motor power increase
                robot.setDrivePower((0.8) * (1 + correction), (0.8) * (1 - correction));

                if (levelTwo) {
                    if (leftDistance >= 48 * 2) {
                        robot.resetDriveEncoders();
                        MOErioAuto.resetError();
                        autoStep++;
                    }
                } else {
                    if (leftDistance >= 48) {
                        robot.resetDriveEncoders();
                        MOErioAuto.resetError();
                        autoStep++;
                    }
                }
                break;

            /*Right side- make a right turning arc*/
            /*Left side- make a left turning arc*/
            case 0:
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                setDrivePowerHands(/*0.5*/0.7,/*0.3*/0.5, correction, LeftSide);

                if (getDistanceLeftInchesHands(LeftSide) >= 72 /*x1*/) {
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                }
                break;

            /*Right side- make a left turning arc*/
            /*Left side- make a right turning arc*/
            /*you should be sideways to the cargo ship*/
            case 1:
                louWizardry = leftDistance - rightDistance / zEffective;
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();
                setDrivePowerHands(0.5, 0.7, correction, LeftSide);

                if (getDistanceRightInchesHands(LeftSide) >= 43 /*x2*/) {
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                }
                break;

            /*roll forward, now in front of hatch*/
            case 2:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower((0.4) * (1 + correction), (0.4) * (1 - correction));

                if (leftDistance >= 41 + 5) {
                    autoStep++;
                    MOErioAuto.resetError();
                    MOErioTurn.resetError();
                }
                break;

            /*Right side- turn 90 degrees to the left*/
            /*Left side- turn 90 degrees to the right*/
            /*turning towards the hatch*/
            case 3:
                MOErioTurn.setHeading(robot.getHeadingDegrees());

                robot.setDrivePower(-0.5 * LeftSide, 0.5 * LeftSide);

                if (reachedHeadingHands(80, -1 * LeftSide)) {
                    MOErioTurn.resetError();
                    autoStep++;
                }
                break;

            /*turning cont'd*/
            case 4:
                MOErioTurn.setHeading(robot.getHeadingDegrees() + 90 * LeftSide);
                correction = MOErioTurn.getCorrection();
                robot.setDrivePower(correction, -correction);

                if ((Math.abs(robot.getHeadingDegrees() + 90 * LeftSide) < 0.5) && (turncounter > 4)) {
                    ++autoStep;
                } else if (Math.abs(robot.getHeadingDegrees() + 90 * LeftSide) < 0.5) {
                    ++turncounter;
                } else {
                    turncounter = 0;
                }
                break;

            /*random step that could've probably gone into case 4 but whatever*/
            case 5:
                autoStep++;
                MOErioAuto.resetError();
                break;

            /*roll towards the hatch*/
            case 6:
                MOErioAuto.setHeading(robot.getHeadingDegrees() + 90 * LeftSide);
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower(0.3 * (1 + correction), 0.3 * (1 - correction));

                if (robot.lidar[0] <= 545 - 25.4 + moementumCorrection) {
                    autoStep++;
                }

                break;

            case 7:
                robot.stopDriving();
                break;
        }
    }
}
