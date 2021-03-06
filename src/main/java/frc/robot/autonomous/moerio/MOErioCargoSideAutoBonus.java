package frc.robot.autonomous.moerio;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.PIDModuleLucy;
import frc.robot.autonomous.GenericAuto;

public class MOErioCargoSideAutoBonus extends GenericAuto {

    PIDModule MOErioAuto = new PIDModule(0.06,0.001,0);
    PIDModuleLucy MOErioTurn = new PIDModuleLucy(2.5e-2, 1.75e-3, 0);
    long startTime = 0;
    double z = 1.33;
    double louWizardry = 0;
    //int LeftSide = 1;
    //-1 is left, 1 is right
    int turncounter = 0;
    double correction =0;
    double moementumCorrection = 100;
    double zEffective;
    boolean levelTwo = false;

    @Override
    public void init() {
        autoStep = -2;
        robot.resetDriveEncoders();
        robot.resetYaw();
        MOErioAuto.resetError();
        MOErioAuto.setHeading(0);
        MOErioTurn.resetError();
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
                robot.setDrivePower((0.8)*(1 + correction),(0.8)*(1 - correction));

                if(levelTwo){
                    if(leftDistance >= 48*2){
                        autoStep++;
                        robot.resetDriveEncoders();
                    }
                } else{
                    if(leftDistance >= 48) {
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

                setDrivePowerHands(/*0.5*/0.7,/*0.3*/0.5,correction,LeftSide);

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
                setDrivePowerHands(0.5,0.7,correction,LeftSide);

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

            /*Right side- turn 90 degrees to the left*/
            /*Left side- turn 90 degrees to the right*/
            /*turning towards the hatch*/
            case 3:
                MOErioTurn.setHeading(robot.getHeadingDegrees());

                robot.setDrivePower(-0.5*LeftSide, 0.5*LeftSide);

                if (reachedHeadingHands(80,-1*LeftSide)) {
                    MOErioTurn.resetError();
                    autoStep++;
                }
                break;

            /*turning cont'd*/
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

            /*random step that could've probably gone into case 4 but whatever*/
            case 5:
                autoStep++;
                MOErioAuto.resetError();
                break;

            /*roll towards the hatch*/
            case 6:
                MOErioAuto.setHeading(robot.getHeadingDegrees()+90*LeftSide);
                correction = MOErioAuto.getCorrection();
                robot.setDrivePower(0.3*(1 + correction),0.3*(1 - correction));

                if(robot.lidar[0] <= 545-25.4+moementumCorrection){
                    autoStep++;
                }
                break;

            /*bonus begins, set z to 1.77*/
            /*bonus- go to the loading station*/
            case 7:
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

                robot.resetDriveEncoders();
                MOErioTurn.resetError();
                MOErioAuto.resetError();

                autoStep++;
                break;

            /*arc backwards to the left, end close and parallel to the rocket*/
            case 8:
                louWizardry = leftDistance - rightDistance / zEffective;

                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                //correction negative, left motor decrease, correction positive, left motor power increase
                setDrivePowerHands(/*-0.15*/-0.25,/*-0.5*/-0.6,correction,LeftSide);

                //LFR                if (Math.abs(getDistanceLeftInchesHands(LeftSide)) >= (57) / z /*x1*/) {
                if (Math.abs(getDistanceRightInchesHands(LeftSide)) >= 196.6/2 /*x1*/) {
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                }
                break;

            /*drive backwards towards the loading station a little*/
            case 9:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(-0.8*(1 - correction), -0.8*(1 + correction));
                if(leftDistance >= 36){
                    autoStep++;
                    MOErioAuto.resetError();
                    MOErioTurn.resetError();
                    robot.resetDriveEncoders();
                }
                break;

            /*right side- turn right 90 degrees, face the wall*/
            /*left side- turn left 90 degrees, face the wall*/
            case 10:
                MOErioTurn.setHeading(robot.getHeadingDegrees());

                robot.setDrivePower(0.5*LeftSide, -0.5*LeftSide);

                if (reachedHeadingHands(80,1*LeftSide)) {
                    MOErioTurn.resetError();
                    autoStep++;
                    turncounter=0;
                }
                break;

            /*turning cont'd*/
            case 11:
                MOErioTurn.setHeading(robot.getHeadingDegrees()-90*LeftSide);
                correction = MOErioTurn.getCorrection();
                robot.setDrivePower(correction,-correction);
//                robot.setDrivePower(Math.signum(correction)*(Math.abs(correction)+0.1),-Math.signum(correction)*(Math.abs(correction)+0.1));

                if ( (Math.abs(robot.getHeadingDegrees()-90*LeftSide) < 0.5) && (turncounter >4) ) {
                    ++autoStep;
                    MOErioAuto.resetError();
                }
                else if (Math.abs(robot.getHeadingDegrees()-90*LeftSide) < 0.5)
                {
                    ++turncounter;
                }
                else {
                    turncounter = 0;
                }
                break;

            /*roll towards the wall*/
            case 12:
                MOErioAuto.setHeading(robot.getHeadingDegrees()-90*LeftSide);
                correction = MOErioAuto.getCorrection();

                //correction negative, left motor decrease, correction positive, left motor power increase
                robot.setDrivePower((0.4)*(1 + correction),(0.4)*(1 - correction));

                if(robot.lidar[0] <= 360+moementumCorrection) {
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                    MOErioTurn.resetError();
                    robot.resetYaw();
                }
                break;

            /*right side- turn right 90 degrees, towards the loading station*/
            /*left side- turn left 90 degrees, towards the loading station*/
            case 13:
                robot.setDrivePower(0.5*LeftSide, -0.5*LeftSide);

                if (reachedHeadingHands(80,1*LeftSide)) {
                    MOErioTurn.resetError();
                    autoStep++;
                }
                break;

            /*turning cont'd*/
            case 14:
                MOErioTurn.setHeading(robot.getHeadingDegrees()-90*LeftSide);
                correction = MOErioTurn.getCorrection();
                robot.setDrivePower(correction,-correction);
//                robot.setDrivePower(Math.signum(correction)*(Math.abs(correction)+0.1),-Math.signum(correction)*(Math.abs(correction)+0.1));

                if ( (Math.abs(robot.getHeadingDegrees()-90*LeftSide) < 0.5) && (turncounter >4) ) {
                    ++autoStep;
                    MOErioAuto.resetError();
                }
                else if (Math.abs(robot.getHeadingDegrees()-90*LeftSide) < 0.5)
                {
                    ++turncounter;
                }
                else {
                    turncounter = 0;
                }
                break;

            /*roll forwards, towards the loading station*/
            case 15:
                MOErioAuto.setHeading(robot.getHeadingDegrees()-90*LeftSide);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower((1.0)*(1 + correction),(1.0)*(1 - correction));

                if(leftDistance >= 108) {
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                }
                break;

            case 16:
                MOErioAuto.setHeading(robot.getHeadingDegrees()-90*LeftSide);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower((0.4)*(1 + correction),(0.4)*(1 - correction));

                if(robot.lidar[0] <= 545+moementumCorrection) {
                    autoStep++;
                    robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                }
                break;

            case 17:
                robot.stopDriving();
                break;
        }
        }
}
