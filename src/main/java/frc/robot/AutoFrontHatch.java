package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.pseudoresonance.pixy2api.Pixy2Line;


//rightmost level 1
public class AutoFrontHatch extends GenericAuto {

    //P = 0.06, I = 0.001, D = 0.0
    PIDModule arcPid = new PIDModule(0.06, 0.001,0.0);
    double z = 1.34;

    //right front hatch s = 72in then 65in for the second arc...
    //left front hatch s = 96in then 50in for the second arc....

    public PixyCam pixyCam = new PixyCam();
    @Override
    public void init() {
        arcPid.resetError();
        arcPid.setHeading(0);
        autoStep = 2;
        robot.resetDriveEncoders();
        robot.resetYaw();
        pixyCam.init();
        pixyCam.run();
        pixyCam.start();

    }

    @Override
    public void run() {
        double leftDistance = robot.getDistanceLeftInches();
        double rightDistance = robot.getDistanceRightInches();
        double louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) / z;

        SmartDashboard.putNumber("error",arcPid.getInput());
        SmartDashboard.putNumber("correction", arcPid.getCorrection());
        SmartDashboard.putNumber("kP", arcPid.pidController.getP());
        SmartDashboard.putNumber("kI", arcPid.pidController.getI());
        SmartDashboard.putNumber("kD", arcPid.pidController.getD());
        SmartDashboard.putNumber("current time", System.currentTimeMillis());
        SmartDashboard.putNumber("the magic", louWizardry);
        SmartDashboard.putNumber("Z", z);
        switch (autoStep) {
            /*case 0:
                arcPid.setHeading(louWizardry) ;
                double correction = arcPid.getCorrection();

                //correction negative, left motor decrease. correction positive, left motor power increase.
                robot.setDrivePower((0.3 ) * (1 + correction),(0.5 * (1 - correction)));


                if(robot.getDistanceLeftInches() >= 72 / z){
                    autoStep++;
                    robot.resetDriveEncoders();
                }
                break;
            case 1:
                louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) * z;
                arcPid.setHeading(louWizardry);
                correction = arcPid.getCorrection();

                //correction negative, left motor decrease. correction positive, left motor power increase.
                robot.setDrivePower((0.5 ) * (1 + correction),(0.3 * (1 - correction)));


                if(robot.getDistanceLeftInches() >= 65){
                    autoStep++;
                    robot.resetDriveEncoders();
                }
                break;*/
            case 2:
                robot.setDrivePower(0.2,0.2);
                if(robot.getDistanceLeftInches() >= 12){
                    autoStep++;
                    robot.resetDriveEncoders();
                }
                /*if(robot.getHeadingDegrees() < 0){
                    robot.turnRightInplace(0.2);
                } else if (robot.getHeadingDegrees() > 0) {
                    robot.turnLeftInplace(0.2);
                } else {
                    autoStep++;
                }
                break;*/
            case 3:
                //get pixycam data
                Pixy2Line.Vector[] vec = pixyCam.getLastVector();
                Pixy2Line.Vector aVec;
                if(vec != null && vec.length > 0){
                    //Print first vector found coords to smartdashboard
                    System.out.println(vec[0].toString());
                    SmartDashboard.putNumber("PixyVec X0", vec[0].getX0());
                    SmartDashboard.putNumber("PixyVec X1", vec[0].getX1());
                    SmartDashboard.putNumber("PixyVec Y0", vec[0].getY0());
                    SmartDashboard.putNumber("PixyVec Y1", vec[0].getY1());
                    aVec = vec[0];
                }else { return; }
                //Pixy resolution 320x300
                //X/2 = 160 with a margin of 40 : 140 < X < 160 is optimal range (NEED TO CALCULATE)
                if(140 < aVec.getX1() && aVec.getX1() < 180){
                    System.out.println("[PixyCam] Robot is in optimal place, Go forward");
                    autoStep++;
                }else{
                    int diff = 160-aVec.getX1();
                    System.out.println("[PixyCam] Robot needs to turn: " + Integer.toString(diff));
                }
            case 4:
                robot.stopDriving();
        }
    }

}

