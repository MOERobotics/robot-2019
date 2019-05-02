package frc.robot.autonomous.unused;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.autonomous.GenericAuto;


//from rightmost position level 1
public class AutoSideHatch extends GenericAuto {

    //P = 0.06, I = 0.001, D = 0.0
    PIDModule arcPid = new PIDModule(0.06, 0.001,0.0);
    double z = 1.4;
    //1.34 for caMOElot
    boolean rightSide = true;

    //front side hatch s = 72in

    @Override
    public void init() {
        arcPid.resetError();
        arcPid.setHeading(0);
        autoStep = -1;
        robot.resetDriveEncoders();
        robot.resetYaw();

    }

    @Override
    public void run() {
        double leftDistance = robot.getDistanceLeftInches();
        double rightDistance = robot.getDistanceRightInches();
        double louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) * z;

        /*
        if(rightSide){
            louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) * z;
        } else {
            louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) / z;
        }
        */


        SmartDashboard.putNumber("error",arcPid.getInput());
        SmartDashboard.putNumber("correction", arcPid.getCorrection());
        SmartDashboard.putNumber("kP", arcPid.pidController.getP());
        SmartDashboard.putNumber("kI", arcPid.pidController.getI());
        SmartDashboard.putNumber("kD", arcPid.pidController.getD());
        SmartDashboard.putNumber("current time", System.currentTimeMillis());
        SmartDashboard.putNumber("the magic", louWizardry);
        SmartDashboard.putNumber("Z", z);
        switch (autoStep) {
            case -1:
                arcPid.setHeading(robot.getHeadingDegrees());
                double correction = arcPid.getCorrection();

                robot.setDrivePower(0.5*(1+correction), 0.3*(1-correction));

                if(robot.getPitchDegrees() >= 3){
                    autoStep++;
                    robot.resetYaw();
                    robot.resetDriveEncoders();
                }
            //left arc
            case 0:
                arcPid.setHeading(louWizardry) ;
                correction = arcPid.getCorrection();

                //correction negative, left motor decrease. correction positive, left motor power increase.
                robot.setDrivePower((0.5 ) * (1 + correction),(0.3 * (1 - correction)));


                if(robot.getDistanceLeftInches() >= 72){
                    autoStep++;
                    SmartDashboard.putNumber("the first arc left", robot.getDistanceLeftInches());
                    SmartDashboard.putNumber("the first arc right", robot.getDistanceRightInches());
                    SmartDashboard.putNumber("the first arc yaw",  robot.getHeadingDegrees());
                    robot.resetDriveEncoders();
                }
                break;
            //right arc
            case 1:
                louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) / z;
                arcPid.setHeading(louWizardry) ;
                correction = arcPid.getCorrection();

                //correction negative, left motor decrease. correction positive, left motor power increase.

                robot.setDrivePower((0.3 ) * (1 + correction),(0.5 * (1 - correction)));


                if(robot.getDistanceLeftInches() >= 60 / z){
                    autoStep++;
                    SmartDashboard.putNumber("the second arc left", robot.getDistanceLeftInches());
                    SmartDashboard.putNumber("the second arc right", robot.getDistanceRightInches());
                    SmartDashboard.putNumber("the second arc yaw", robot.getHeadingDegrees());
                    arcPid.setHeading(0);
                    robot.resetDriveEncoders();
                    robot.resetYaw();
                }
                break;
            /*
            case 2:
                arcPid.setHeading(robot.getHeadingDegrees());
                correction = arcPid.getCorrection();

                robot.setDrivePower(0.4 * (1 + correction), 0.4 * (1 - correction));

                if(robot.getDistanceLeftInches() >= 45+14){
                    autoStep++;
                    robot.resetYaw();
                    robot.resetDriveEncoders();
                }
                break;
            */
            /*
            case 3:
                if(robot.getHeadingDegrees() <= -90) {
                    autoStep++;
                    robot.resetYaw();
                    robot.resetDriveEncoders();
                    arcPid.setHeading(0);
                } else if(robot.getHeadingDegrees() <= -80){
                    robot.turnLeftInplace(0.2);
                } else{
                    robot.turnLeftInplace(0.4);
                }

                break;
            */
            /*
            case 4:
                arcPid.setHeading(robot.getHeadingDegrees());
                correction = arcPid.getCorrection();

                robot.setDrivePower(0.4 * (1 + correction), 0.4 * (1 - correction));

                //probably gonna have to use lidar for this because it keeps varying
                if(robot.getDistanceLeftInches() >= 21){
                    autoStep++;
                    robot.resetYaw();
                    robot.resetDriveEncoders();
                }
                break;
            */
            /*
            case 5:
                if(robot.getHeadingDegrees() <= -90) {
                    autoStep++;
                    robot.resetYaw();
                    robot.resetDriveEncoders();
                    arcPid.setHeading(0);
                } else if(robot.getHeadingDegrees() <= -80){
                    robot.turnLeftInplace(0.2);
                } else{
                    robot.turnLeftInplace(0.4);
                }
                break;

            case 6:
                louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) / z;
                arcPid.setHeading(louWizardry) ;
                correction = arcPid.getCorrection();

                //correction negative, left motor decrease. correction positive, left motor power increase.
                robot.setDrivePower((0.3 ) * (1 + correction),(0.5 * (1 - correction)));


                if(robot.getDistanceLeftInches() >= 96 / z){
                    autoStep++;
                    arcPid.setHeading(0);
                    robot.resetDriveEncoders();
                    robot.resetYaw();
                }
                break;

            case 7:
                louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) * z;
                arcPid.setHeading(louWizardry);
                correction = arcPid.getCorrection();

                //correction negative, left motor decrease. correction positive, left motor power increase.
                robot.setDrivePower((0.5 ) * (1 + correction),(0.3 * (1 - correction)));


                if(robot.getDistanceLeftInches() >= 96){
                    autoStep++;
                    robot.resetDriveEncoders();
                }
                break;
            */

            case 2:
                robot.stopDriving();
        }
    }

}

