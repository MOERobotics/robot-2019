package frc.robot.autonomous.moerio;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.PIDModuleLucy;
import frc.robot.autonomous.GenericAuto;

public class MOErioCargoFrontAutoBonus extends GenericAuto {
    //line this up on the far side of the HAB space, against level three.
    PIDModule MOErioAuto = new PIDModule(0.06, 0.001, 0);
    PIDModuleLucy MOErioTurn = new PIDModuleLucy(2.5e-2, 1.75e-3, 0);
    long startTime = 0;
    double z = 1.81;
    double louWizardry = 0;
    //int LeftSide = 1;
    int turncounter = 0;
    boolean levelTwo = false;
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
    }

    @Override
    public void printSmartDashboard() {

        SmartDashboard.putNumber("Error: ", MOErioAuto.getInput());
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
        SmartDashboard.putNumber("Arc Lengths: ", 49);
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
                    autoStep=-1;
                }
                break;

            /*roll off the HAB*/
            case -1:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                double correction = MOErioAuto.getCorrection();

                //correction negative, left motor decrease, correction positive, left motor power increase
                robot.setDrivePower((0.8)*(1 + correction),(0.8)*(1 - correction));
                /*
                if (robot.getPitch() >= 3) {
                    autoStep++;
                    robot.resetDriveEncoders();
                }
                */
                if(levelTwo){
                    if(leftDistance >= (46-8)*2){
                        autoStep++;
                        robot.resetDriveEncoders();
                    }
                } else if(leftDistance >= 46-8) {
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

                //correction negative, left motor decrease, correction positive, left motor power increase
                setDrivePowerHands(0.1,0.5,correction,LeftSide);

//LFR                if (Math.abs(getDistanceLeftInchesHands(LeftSide)) >= (57) / z /*x1*/) {
                if (Math.abs(getDistanceRightInchesHands(LeftSide)) >= (51.0) /*x1*/) {
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

                setDrivePowerHands(0.5, 0.1,correction,LeftSide);

                if (Math.abs(getDistanceLeftInchesHands(LeftSide)) >= 51.0){
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
//LFR                robot.setDrivePower(correction*LeftSide,-correction*LeftSide);
                robot.setDrivePower(correction,-correction);

                if ( (Math.abs(robot.getHeadingDegrees()) < 0.5) && (turncounter >4) ) {
                    ++autoStep;
                }
                else if (Math.abs(robot.getHeadingDegrees()) < 0.5)
                {
                    ++turncounter;
                }
                else {
                    turncounter = 0;
                }
                break;

            /*roll forward towards the hatch*/
            case 3:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower((0.3)*(1 + correction),(0.3)*(1 - correction));

                if (robot.lidar[0] <= 605+moementumCorrection) {
                    autoStep++;
                }

                // LFR -- Huh?
                /*if(leftDistance >= 545/24.5){
                    autoStep++;
                }*/
                break;

            /*bonus begins, setting z to 1.77, goal: loading station*/
            case 4:
                robot.stopDriving();
                z = 1.77;
                if (LeftSide==-1)
                {
                    zEffective = 1/z;
                }
                else
                {
                    zEffective = z;
                }
                autoStep++;
                robot.resetDriveEncoders();
                MOErioAuto.resetError();
                MOErioTurn.resetError();
                break;

            /*right side- arc backwards to the left*/
            /*left side- arc backwards to the right*/
            case 5:

                louWizardry = leftDistance - rightDistance / zEffective;

                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                setDrivePowerHands(/*-0.15*/-0.25,/*-0.5*/-0.6,correction,LeftSide);

    //LFR                if (Math.abs(getDistanceLeftInchesHands(LeftSide)) >= (57) / z /*x1*/) {
                if (Math.abs(getDistanceRightInchesHands(LeftSide)) >= 196.6/2 /*x1*/) {
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                }
                break;

            /*roll forwards, towards the wall*/
            case 6:
                MOErioAuto.setHeading(robot.getHeadingDegrees()-90*LeftSide);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower((0.6)*(1 + correction),(0.6)*(1 - correction));

                if(leftDistance >= 144) {
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                    MOErioTurn.resetError();
                    robot.resetYaw();
                }
                break;

            case 7:
                MOErioAuto.setHeading(robot.getHeadingDegrees()-90*LeftSide);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower((0.4)*(1 + correction),(0.4)*(1 - correction));

                if(robot.lidar[0] <= 360+moementumCorrection) {
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                    MOErioTurn.resetError();
                }
                break;

            /*straightening out by turning in place*/
            case 8:
                MOErioTurn.setHeading(robot.getHeadingDegrees()-90*LeftSide);
                correction = MOErioTurn.getCorrection();
//LFR                robot.setDrivePower(correction*LeftSide,-correction*LeftSide);
                robot.setDrivePower(correction,-correction);

                if ( (Math.abs(robot.getHeadingDegrees()) < 0.5) && (turncounter >4) ) {
                    ++autoStep;
                    robot.resetYaw();
                    MOErioTurn.resetError();
                    MOErioAuto.resetError();
                    robot.resetDriveEncoders();
                }
                else if (Math.abs(robot.getHeadingDegrees()) < 0.5)
                {
                    ++turncounter;
                }
                else {
                    turncounter = 0;
                }
                break;

            /*right side- turn 90 degrees to the right*/
            /*left side-turn 90 degrees to the left*/
            /*turning towards the loading station*/
            case 9:

                robot.setDrivePower(0.5*LeftSide, -0.5*LeftSide);

                if (reachedHeadingHands(80,1*LeftSide)) {
                    MOErioTurn.resetError();
                    autoStep++;
                }
                break;

            /*turning cont'd*/
            case 10:
                MOErioTurn.setHeading(robot.getHeadingDegrees()-90);
                correction = MOErioTurn.getCorrection();
                robot.setDrivePower(correction,-correction);
//                robot.setDrivePower(Math.signum(correction)*(Math.abs(correction)+0.1),-Math.signum(correction)*(Math.abs(correction)+0.1));

                if ( (Math.abs(robot.getHeadingDegrees()-90) < 0.5) && (turncounter >4) ) {
                    ++autoStep;
                    MOErioAuto.resetError();
                }
                else if (Math.abs(robot.getHeadingDegrees()-90) < 0.5)
                {
                    ++turncounter;
                }
                else {
                    turncounter = 0;
                }
                break;

            /*roll forwards, towards the loading station*/
            case 11:
                MOErioAuto.setHeading(robot.getHeadingDegrees()-90*LeftSide);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower((1.0)*(1 + correction),(1.0)*(1 - correction));

                if(leftDistance >= 108) {
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                }
                break;

            case 12:
                MOErioAuto.setHeading(robot.getHeadingDegrees()-90*LeftSide);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower((0.4)*(1 + correction),(0.4)*(1 - correction));

                if(robot.lidar[0] <= 545+moementumCorrection) {
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                }
                break;

            case 13:
                robot.stopDriving();
                break;

        }
    }
}
