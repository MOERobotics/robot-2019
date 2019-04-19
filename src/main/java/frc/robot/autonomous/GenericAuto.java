package frc.robot.autonomous;

import frc.robot.PIDModule;
import frc.robot.genericrobot.GenericRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class GenericAuto {
    public int autoStep = 0;
    public abstract void init();
    public abstract void run();
    public GenericRobot robot;
    public int LeftSide = 1;
    public int lastStep = 42;
    public boolean levelTwo;
    public int habLevel;

    public double turnPower = /*0.2*/0.28;
    public double higherTurnPower = /*0.25*/0.33;

    public boolean withinElevatorTolerance = false;
    public boolean withinArmTolerance = false;

    private boolean haveWeYelledAtTheCoderYet = false;
    public void printSmartDashboard() {
        if(haveWeYelledAtTheCoderYet == false) {
            System.err.println("Help us God");
            haveWeYelledAtTheCoderYet = true;
        }
    }

    //Raising the elevator until position is reached
    public void raiseElevator(double position, PIDModule elevatorPID){
        if(robot.getElevatorEncoderCount() > position){
            withinElevatorTolerance = true;
            elevatorPID.resetError();
        }else{
            robot.driveElevator(0.7);
        }
    }

    /*public void lowerElevator(double position, PIDModule elevatorPID) {
        robot.driveElevator(-0.3);
        if(robot.getElevatorEncoderCount()  <= position){
            autoStep++;
            elevatorPID.resetError();
        }
    }*/

    //PID controlling the elevator
    public void PIDElevator(double position, PIDModule elevatorPID){
        elevatorPID.setHeading(robot.getElevatorEncoderCount() - position);
        double elevatorCorrection = elevatorPID.getCorrection();

        robot.driveElevator(elevatorCorrection);
    }

    //Raising the arm until position is reached
    public void raiseArm(double position, PIDModule armPID){
        if(robot.getArmEncoderCount() < position){
            robot.driveArm(0.4);
        } else{
            withinArmTolerance = true;
            armPID.resetError();
        }
    }

    //PID controlling the arm
    public void PIDArm(double position, PIDModule armPID){
        armPID.setHeading(robot.getArmEncoderCount() - position);
        double armCorrection = armPID.getCorrection();

        robot.driveArm(armCorrection);
    }

    public void setDrivePowerHands(double left, double right, double correction, int Handedness) {
        if (!(Handedness == -1)) {
            robot.setDrivePower(left * (1 + correction), right * (1 - correction));
        } else {
            robot.setDrivePower(right * (1 + correction), left * (1 - correction));
        }
    }

    public double getDistanceLeftInchesHands(int Handedness) {
        if (!(Handedness == -1)) {
            return (Math.abs(robot.getDistanceLeftInches()));
        } else {
            return (Math.abs(robot.getDistanceRightInches()));
        }
    }

    public double getDistanceRightInchesHands(int Handedness) {
        if (!(Handedness == -1)) {
            return (Math.abs(robot.getDistanceRightInches()));
        } else {
            return (Math.abs(robot.getDistanceLeftInches()));
        }
    }

    //pass in degrees and direction
    //1 = to the right
    //-1 = to the left
    public boolean reachedHeadingHands(double degrees, int Handedness) {
        if (Handedness == 1) {
            if (robot.getHeadingDegrees() >= degrees) {
                return true;
            }
        } else if (Handedness == -1) {
            if (robot.getHeadingDegrees() <= degrees * Handedness) {
                return true;
            }
        } else {
            return false;
        }
        return false;
    }
}
