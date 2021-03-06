package frc.robot.autonomous.moerio;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.PIDModuleLucy;
import frc.robot.autonomous.GenericAuto;

/*what is happening*/
public class MOErioCargoSideAuto extends GenericAuto {

    PIDModule MOErioAuto = new PIDModule(0.06,0.001,0);
    PIDModuleLucy MOErioTurn = new PIDModuleLucy(2.5e-2, 1.75e-3, 0);
    long startTime = 0;
    double z = 1.33;
    double louWizardry = 0;
    //int LeftSide = -1;
    //-1 is left, 1 is right
    int turncounter = 0;
    double correction =0;
    double moementumCorrection = 50;
    double zEffective;
    boolean levelTwo = true;

    @Override
    public void init() {
        autoStep = -2;
        robot.resetDriveEncoders();
        robot.resetYaw();
        MOErioAuto.resetError();
        MOErioAuto.setHeading(0);
        startTime = System.currentTimeMillis();

        if (LeftSide==-1)
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
        //SmartDashboard.putNumber("Left Side", LeftSide);
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
            case -1:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();

                //correction negative, left motor decrease, correction positive, left motor power increase
                robot.setDrivePower((0.5)*(1 + correction),(0.5)*(1 - correction));

                if(levelTwo){
                    if(leftDistance >= 48*2){
                        autoStep++;
                        robot.resetDriveEncoders();
                    }
                } else{
                    if(leftDistance >= 48) {
                    /* Let's remove this once we've tested left and right sides.
                    if (LeftSide) {
                        autoStep = 5;
                    } else {
                        autoStep=0;
                    }
                */
                    autoStep++;
                    robot.resetDriveEncoders();
                    }
                }

                break;
            /*Right side- make a right turning arc*/
            /*Left side- make a left turning arc*/
            case 0:
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                //correction negative, left motor decrease, correction positive, left motor power increase
                //robot.setDrivePower((0.5)*(1 + correction),(0.3)*(1 - correction));
                setDrivePowerHands(0.5,0.3,correction,LeftSide);

                if (getDistanceLeftInchesHands(LeftSide) >= 72 /*x1*/) {
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                }
                break;
            /*Right side- make a left turning arc*/
            /*Left side- make a right turning arc*/
            case 1:
                louWizardry = leftDistance - rightDistance / zEffective;
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();
//                robot.setDrivePower((0.3)*(1 + correction),(0.5)*(1 - correction));
                setDrivePowerHands(0.3,0.5,correction,LeftSide);


                if (getDistanceRightInchesHands(LeftSide) >= 43 /*x2*/) {
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                }
                break;
            case 2:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower((0.4)*(1 + correction),(0.4)*(1 - correction));

                if (leftDistance >= 41+5) {
                    autoStep++;
                    MOErioAuto.resetError();
                    MOErioTurn.resetError();
                }
                break;
            /*
            case 3:
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
                }
                robot.setDrivePower(-0.45*LeftSide,0.45*LeftSide);
                //robot.turnLeftInplace(0.45);
                if(robot.getHeadingDegrees() <= -80*LeftSide){
                    autoStep++;
                }
                break;
            case 4:
                robot.setDrivePower(-0.3*LeftSide,0.3*LeftSide);
                //robot.turnLeftInplace(0.3);
                if(robot.getHeadingDegrees() <= -90*LeftSide){
                    autoStep++;
                }
                break;
            case 5:
                MOErioAuto.setHeading(robot.getHeadingDegrees() + 90*LeftSide);
                correction = MOErioAuto.getCorrection();

                setDrivePowerHands(1.0,-1.0, correction, LeftSide);

                if(Math.abs(robot.getHeadingDegrees()+90*LeftSide) < 0.5){
                    robot.setDrivePower(0,0);
                    autoStep++;
                    robot.resetYaw();
                    MOErioAuto.resetError();
                }
                break;*/
            case 3:
                MOErioTurn.setHeading(robot.getHeadingDegrees());

                robot.setDrivePower(-0.5*LeftSide, 0.5*LeftSide);

                if (reachedHeadingHands(80,-1*LeftSide)) {
                    MOErioTurn.resetError();
                    autoStep++;
                }
                break;
            case 4:
                MOErioTurn.setHeading(robot.getHeadingDegrees()+90*LeftSide);
                correction = MOErioTurn.getCorrection();
                robot.setDrivePower(correction,-correction);

                if ( (Math.abs(robot.getHeadingDegrees()+90*LeftSide) < 0.5) && (turncounter >4) ) {
                    ++autoStep;
                }
                else if (Math.abs(robot.getHeadingDegrees()+90*LeftSide) < 0.5)
                {
                    ++turncounter;
                }
                else {
                    turncounter = 0;
                }
                break;
            case 5:
                autoStep++;
                MOErioAuto.resetError();
                break;
            case 6:
                MOErioAuto.setHeading(robot.getHeadingDegrees()+90*LeftSide);
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower(0.3*(1 + correction),0.3*(1 - correction));

                if(robot.lidar[0] <= 545-25.4+moementumCorrection){
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

        }
        }
}
