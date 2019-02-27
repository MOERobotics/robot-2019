package frc.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.swing.plaf.basic.BasicSplitPaneUI;

public class MOErioCargoSideAuto extends GenericAuto {

    PIDModule MOErioAuto = new PIDModule(0.06,0.001,0);
    long startTime = 0;
    double z = 1.33;
    double louWizardry = 0;
    boolean LeftSide = false;
    double correction =0;
    double moementumCorrection = 147;
    double zEffective;


    public void setDrivePowerHands(double left, double right, double correction, boolean Handedness) {
        if (!Handedness)
        {
            robot.setDrivePower(left*(1+correction),right*(1-correction));
        }
        else
        {
            robot.setDrivePower(right * (1 + correction), left * (1 - correction));
        }
    }

    public double getDistanceLeftInchesHands(boolean Handedness) {
        if (!Handedness) {
            return (robot.getDistanceLeftInches());
        } else {
            return (robot.getDistanceRightInches());
        }
    }
    public double getDistanceRightInchesHands(boolean Handedness) {
        if (!Handedness) {
            return (robot.getDistanceRightInches());
        } else {
            return (robot.getDistanceLeftInches());
        }
    }


    @Override
    public void init() {
        autoStep = -2;
        robot.resetDriveEncoders();
        robot.resetYaw();
        MOErioAuto.resetError();
        MOErioAuto.setHeading(0);
        startTime = System.currentTimeMillis();

        if (LeftSide)
        {
            zEffective = 1/z;
        }
        else
        {
            zEffective = z;
        }

    }

    @Override
    public void printSmartDashboard() {
        correction = MOErioAuto.getCorrection();
        SmartDashboard.putNumber("Error: ", MOErioAuto.getInput());
        SmartDashboard.putNumber("Correction: ",correction);
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
        louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) * zEffective;

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
                correction = MOErioAuto.getCorrection();

                //correction negative, left motor decrease, correction positive, left motor power increase
                robot.setDrivePower((0.5)*(1 + correction),(0.5)*(1 - correction));

                if(Math.abs(robot.getDistanceLeftInches()) >= 48) {
                    /* Let's remove this once we've tested left and right sides.
                    if (LeftSide) {
                        autoStep = 5;
                    } else {
                        autoStep=0;
                    }
                */
                    robot.resetDriveEncoders();
                }
                break;
            case 0:
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                //correction negative, left motor decrease, correction positive, left motor power increase
                //robot.setDrivePower((0.5)*(1 + correction),(0.3)*(1 - correction));
                setDrivePowerHands(0.5,0.3,correction,LeftSide);

                /* long now = System.currentTimeMillis();
                long endTime = startTime ;
                MOErioArc.setHeading(robot.getHeadingDegrees() - (dT * yawRate));*/

                if (Math.abs(getDistanceLeftInchesHands(LeftSide)) >= 72 /*x1*/) {
                    autoStep++;
                    robot.resetDriveEncoders();
                }
                break;
            case 1:
                louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) / zEffective;
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();
//                robot.setDrivePower((0.3)*(1 + correction),(0.5)*(1 - correction));
                setDrivePowerHands(0.3,0.5,correction,LeftSide);


                if (Math.abs(getDistanceLeftInchesHands(LeftSide) >= 43 / zEffective /*x2*/) {
                    autoStep++;
                    robot.resetDriveEncoders();
                }
                break;
            case 2:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower((0.4)*(1 + correction),(0.4)*(1 - correction));

                if (Math.abs(robot.getDistanceLeftInches()) >= 41+5) {
                    autoStep++;
                    MOErioAuto.resetError();
                }
                break;
            case 3:
                /*
                MOErioAuto.setHeading((robot.getHeadingDegrees() - -90)*0.9);
                correction = MOErioAuto.getCorrection();
                double temp_correction = correction * -0.5;
                if (temp_correction > 0) temp_correction += 0.20;
                if (temp_correction < 0) temp_correction -= 0.20;
                robot.turnLeftInplace(temp_correction);
                if (robot.getHeadingDegrees() <= -87 &&
                    robot.getHeadingDegrees() >= -93 &&
                    correction <  0.3 &&
                    correction > -0.3
                ) {
                    autoStep++;
                }*/
                robot.turnLeftInplace(0.45);
                if(robot.getHeadingDegrees() <= -80){
                    autoStep++;
                }
                break;
            case 4:
                robot.turnLeftInplace(0.3);
                if(robot.getHeadingDegrees() <= -90){
                    autoStep++;
                }
                break;
            case 5:
                MOErioAuto.setHeading(robot.getHeadingDegrees() + 90);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(1.0*(correction),-1.0*correction);

                if(Math.abs(robot.getHeadingDegrees()+90) < 0.5){
                    robot.setDrivePower(0,0);
                    autoStep++;
                    robot.resetYaw();
                    MOErioAuto.resetError();
                }
                break;
            case 6:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower(0.3*(1 + correction),0.3*(1 - correction));

                if(robot.lidar[0] <= 545+moementumCorrection){
                    autoStep++;
                }

                break;
                /*robot.resetDriveEncoder();
                robot.setDrivePower((0.3)*(1 + correction),(0.3)*(1 - correction));

                if (robot.getDistanceLeftInches() == /*x4) {
                    robot.stopDriving();
                }*/
            case 7:
                robot.stopDriving();
                break;
            case 8:
                louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) / z;
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower((0.3)*(1 + correction),(0.5)*(1 - correction));

                if (Math.abs(robot.getDistanceLeftInches()) >= 72 / z /*x1*/) {
                    autoStep++;
                    robot.resetDriveEncoders();
                }
                break;
            case 9:
                louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) * z;
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower((0.5)*(1 + correction),(0.3)*(1 - correction));

                if (Math.abs(robot.getDistanceLeftInches()) >= 72  /*x2*/) {
                    autoStep++;
                    robot.resetDriveEncoders();
                }
                break;
            case 10:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower((0.4)*(1 + correction),(0.4)*(1 - correction));

                if (Math.abs(robot.getDistanceLeftInches()) >= 41+5) {
                    autoStep++;
                    robot.resetYaw();
                }
            case 11:
                MOErioAuto.setHeading((robot.getHeadingDegrees() - 90)*0.9);
                correction = MOErioAuto.getCorrection();
                //je ne sais pas
                /*
                temp_correction = correction * -0.5;
                if (temp_correction > 0) temp_correction += 0.20;
                if (temp_correction < 0) temp_correction -= 0.20;
                robot.turnRightInplace(temp_correction);
                if (robot.getHeadingDegrees() >= 87 &&
                        robot.getHeadingDegrees() <= 93 &&
                        correction <  0.3 &&
                        correction > -0.3
                ) {
                    autoStep=4;
                }
                break;*/

        }
        }
}
