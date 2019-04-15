package frc.robot.autonomous.moerio;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.PIDModuleLucy;
import frc.robot.autonomous.GenericAuto;

public class MOErioCargoFrontAuto extends GenericAuto {
    //line this up on the far side of the HAB space.
    //1.67 for 0.5 0.15 power
    //1.33 for 0.5 0.3 power
    PIDModule MOErioAuto = new PIDModule(0.06, 0.001, 0);
    PIDModuleLucy MOErioTurn = new PIDModuleLucy(2.5e-2, 1.75e-3, 0);
    long startTime = 0;
    double z = 1.81;
    double louWizardry = 0;
    //int LeftSide = -1;
    int turncounter = 0;
    boolean levelTwo = false;
    double moementumCorrection = 50;
    double zEffective;

    public void setDrivePowerHands(double left, double right, double correction, int Handedness) {
        if (!(Handedness==-1))
        {
            robot.setDrivePower(left*(1 + correction),right*(1 - correction));
        }
        else
        {
            robot.setDrivePower(right * (1 + correction), left * (1 - correction));
        }
    }

    public double getDistanceLeftInchesHands(int Handedness) {
        if (!(Handedness==-1)) {
            return (Math.abs(robot.getDistanceLeftInches()));
        } else {
            return (Math.abs(robot.getDistanceRightInches()));
        }
    }
    public double getDistanceRightInchesHands(int Handedness) {
        if (!(Handedness==-1)) {
            return (Math.abs(robot.getDistanceRightInches()));
        } else {
            return (Math.abs(robot.getDistanceLeftInches()));
        }
    }

    @Override
    public void init() {
        autoStep = -2;
        robot.resetDriveEncoders();
        robot.resetYaw();
        MOErioAuto.setHeading(0);
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
        SmartDashboard.putNumber("i am so angry right now", 6);
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
            case -1:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                double correction = MOErioAuto.getCorrection();

                //correction negative, left motor decrease, correction positive, left motor power increase
                robot.setDrivePower((0.5)*(1 + correction),(0.5)*(1 - correction));
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
            case 1:
                louWizardry = leftDistance - rightDistance * zEffective;
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                setDrivePowerHands(0.5, 0.1,correction,LeftSide);

                if (Math.abs(getDistanceLeftInchesHands(LeftSide)) >= 51.0){
                    autoStep++;
                    //robot.resetDriveEncoders();
                    MOErioAuto.resetError();
                    MOErioTurn.resetError();
                }
                break;
                //59.4 in 121 t
            /*case 2:
                SmartDashboard.putString("got to step two", "yee");
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(LeftSide*1.0*(correction*0.5),LeftSide*-1.0*correction*0.5);

                if(Math.abs(robot.getHeadingDegrees()) < 0.5 && startTime >= System.currentTimeMillis()+1000){
                    SmartDashboard.putString("got to step two", "yee");
                    robot.setDrivePower(0,0);
                    autoStep++;
                    MOErioAuto.resetError();
                }
                break;*/
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
            case 3:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower((0.3)*(1 + correction),(0.3)*(1 - correction));

                if (robot.lidar[0] <= 545+moementumCorrection) {
                    autoStep++;
                }

                // LFR -- Huh?
                /*if(leftDistance >= 545/24.5){
                    autoStep++;
                }*/
                break;
            case 4:
                robot.stopDriving();
                break;









            /*LEFT SIDE CASES (RETIRED)*/
            case 9:
                louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) * z;
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower((0.5)*(1 + correction), (0.15)*(1 - correction));

                if (Math.abs(robot.getDistanceLeftInches()) >= 53){
                    autoStep++;
                    robot.resetDriveEncoders();
                }
                break;
            case 10:
                louWizardry = Math.abs(leftDistance) - Math.abs(rightDistance) / z;
                MOErioAuto.setHeading(louWizardry);
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower((0.15)*(1 + correction), (0.5)*(1 - correction));

                if (Math.abs(robot.getDistanceLeftInches()) >= 53/z){
                    autoStep = 2;
                    robot.resetDriveEncoders();
                }
                break;
        }
    }
}
