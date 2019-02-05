package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


//from rightmost position level 1
public class AutoSideHatch extends GenericAuto {

    //P = 0.06, I = 0.001, D = 0.0
    PIDModule arcPid = new PIDModule(0.06, 0.001,0.0);
    double z = 1.34;
    boolean rightSide = true;

    //front side hatch s = 72in

    @Override
    public void init() {
        arcPid.resetError();
        arcPid.setHeading(0);
        autoStep = 0;
        robot.resetDriveEncoder();
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
            case 0:
                arcPid.setHeading(louWizardry) ;
                double correction = arcPid.getCorrection();

                //correction negative, left motor decrease. correction positive, left motor power increase.
                robot.setDrivePower((0.5 ) * (1 + correction),(0.3 * (1 - correction)));


                if(robot.getDistanceLeftInches() >= 72){
                    autoStep++;
                    robot.resetDriveEncoder();
                }
                break;
            case 1:
                louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) / z;
                arcPid.setHeading(louWizardry) ;
                correction = arcPid.getCorrection();

                //correction negative, left motor decrease. correction positive, left motor power increase.
                robot.setDrivePower((0.3 ) * (1 + correction),(0.5 * (1 - correction)));


                if(robot.getDistanceLeftInches() >= 72 / z){
                    autoStep++;
                    arcPid.setHeading(0);
                    robot.resetDriveEncoder();
                    robot.resetYaw();
                }
                break;

            case 2:
                arcPid.setHeading(robot.getHeadingDegrees());
                correction = arcPid.getCorrection();

                robot.setDrivePower(0.4 * (1 + correction), 0.4 * (1 - correction));

                if(robot.getDistanceLeftInches() >= 45+14){
                    autoStep++;
                    robot.resetYaw();
                    robot.resetDriveEncoder();
                }
                break;


            case 3:
                if(robot.getHeadingDegrees() <= -90) {
                    autoStep++;
                    robot.resetYaw();
                    robot.resetDriveEncoder();
                    arcPid.setHeading(0);
                } else if(robot.getHeadingDegrees() <= -80){
                    robot.turnLeftInplace(0.2);
                } else{
                    robot.turnLeftInplace(0.4);
                }

                break;


            case 4:
                arcPid.setHeading(robot.getHeadingDegrees());
                correction = arcPid.getCorrection();

                robot.setDrivePower(0.4 * (1 + correction), 0.4 * (1 - correction));

                //probably gonna have to use lidar for this because it keeps varying
                if(robot.getDistanceLeftInches() >= 21){
                    autoStep++;
                    robot.resetYaw();
                    robot.resetDriveEncoder();
                }
                break;

            /*
            case 5:
                if(robot.getHeadingDegrees() <= -90) {
                    autoStep++;
                    robot.resetYaw();
                    robot.resetDriveEncoder();
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
                    robot.resetDriveEncoder();
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
                    robot.resetDriveEncoder();
                }
                break;
            */

            case 5:
                robot.stopDriving();
        }
    }

}

