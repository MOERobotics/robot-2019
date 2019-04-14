package frc.robot.genericrobot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Bench extends GenericRobot{

    @Override
    public int numSensors() {
        return 0;
    }

    @Override
    public double getDistanceLeftInches() {
        return 0;
    }

    @Override
    public double getDistanceRightInches() {
        return 0;
    }

    @Override
    public double getHeadingDegrees() {
        return 0;
    }

    @Override
    public double getPitchDegrees() {
        return 0;
    }

    @Override
    public double getRollDegrees() {
        return 0;
    }

    @Override
    public double getElevatorEncoderCountInternal() {
        return 0;
    }

    @Override
    public double getArmEncoderCountInternal() {
        return 0;
    }

    @Override
    public void resetDriveEncoders() {

    }

    @Override
    public void resetYaw() {

    }

    @Override
    public void checkSafety() {

    }

    @Override
    public void resetArmPosition() {

    }

    @Override
    public void resetElevatorPosition() {

    }

    @Override
    public void resetClimberPosition() {

    }

    @Override
    protected void setDrivePowerInternal(double leftMotor, double rightMotor) {

    }

    @Override
    public void shiftDriveInternal(boolean value) {

    }

    @Override
    protected void setElevatorInternal(double power) {

    }

    @Override
    public boolean atElevatorTopLimit() {
        return false;
    }

    @Override
    public boolean atElevatorBottomLimit() {
        return false;
    }

    @Override
    protected void setArmInternal(double power) {

    }

    @Override
    protected void setRollerInternal(double power) {

    }

    @Override
    public void shiftSpearShaftInternal(boolean out) {

    }

    @Override
    public void shiftSpearHookInternal(boolean out) {

    }

    @Override
    public void climbInternal(double power) {

    }

    @Override
    public double getClimberLEncoderCount() {
        return 0;
    }

    @Override
    public double getClimberREncoderCount() {
        return 0;
    }

    @Override
    public void climbLDown(double power) {

    }

    @Override
    public void climbRDown(double power) {

    }

    @Override
    public void linearSlideInternal(DoubleSolenoid.Value state) {

    }

    @Override
    public void getClimberCurrent() {

    }

    @Override
    public void footSpacerCylinderInternal(boolean state) {

    }
}
