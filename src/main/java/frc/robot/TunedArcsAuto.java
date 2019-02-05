package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class TunedArcsAuto extends GenericAuto {

    //P = 0.06, I = 0.001, D = 0.0
    PIDModule arcPid = new PIDModule(0.06, 0.001,0.0);
    double z = 1.34;

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
                }
                break;
            case 2:
                robot.stopDriving();
        }
    }

}

