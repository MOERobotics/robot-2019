package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


//from rightmost position level 1
public class AutoRocket extends GenericAuto {

    //P = 0.06, I = 0.001, D = 0.0
    PIDModule arcPid = new PIDModule(0.06, 0.001,0.0);
    double z = 1.11;

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
                robot.setDrivePower((0.5 ) * (1 + correction),(0.4 * (1 - correction)));


                if(robot.getDistanceLeftInches() >= 96){
                    autoStep++;
                    robot.resetDriveEncoder();
                }
                break;

            case 1:
                if(robot.getHeadingDegrees() > 0){
                    robot.turnLeftInplace(0.2);
                } else {
                    autoStep++;
                }
                break;

            case 2:
                robot.stopDriving();
        }
    }

}

