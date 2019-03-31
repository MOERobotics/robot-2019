package frc.robot.genericrobot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.*;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.genericrobot.GenericRobot;

public class MOErio extends GenericRobot {

    final double TICKS_TO_INCHES = 44;

    TalonSRX driveLA = new TalonSRX(0) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveLB = new TalonSRX(15) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveRA = new TalonSRX(1) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveRB = new TalonSRX(14) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX elevator = new TalonSRX(2) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX rollLeft = new TalonSRX(12) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX rollRight = new TalonSRX(3) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX wrist = new TalonSRX(4) {{setNeutralMode(NeutralMode.Brake);}};

    Encoder encoderL        = new Encoder(0, 1, false, CounterBase.EncodingType.k1X);
    Encoder encoderR        = new Encoder(2, 3, false, CounterBase.EncodingType.k1X);
    Encoder encoderElevator = new Encoder(4, 5, true,  CounterBase.EncodingType.k2X);

    @Override
    public void footSpacerCylinderInternal(boolean value) {

    }

    Encoder encoderWrist    = new Encoder(8, 9, true,  CounterBase.EncodingType.k2X);

    DigitalInput elevatorBottomLimitSwitch = new DigitalInput(6);
    DigitalInput elevatorTopLimitSwitch    = new DigitalInput(7);

    @Override
    public int numSensors() {
        return 1;
    }

    @Override
    public void shiftSpearShaftInternal(boolean out) {

    }

    @Override
    public void shiftSpearHookInternal(boolean out) {

    }

    AHRS navx = new AHRS(SPI.Port.kMXP,(byte) 50);

    {
        driveRA.setInverted(true);
        driveRB.setInverted(true);
        rollLeft.setInverted(true);
    }

    @Override
    public void setDrivePowerInternal(double leftMotor, double rightMotor) {
        driveLA.set(ControlMode.PercentOutput,  leftMotor);
        driveLB.set(ControlMode.PercentOutput,  leftMotor);

        driveRA.set(ControlMode.PercentOutput, rightMotor);
        driveRB.set(ControlMode.PercentOutput, rightMotor);
    }

    @Override
    public void shiftDriveInternal(boolean value) {

    }

    @Override
    protected void setElevatorInternal(double power) {
        elevator.set(ControlMode.PercentOutput, power);
    }

    @Override
    protected void setArmInternal(double power) {
        wrist.set(ControlMode.PercentOutput, power);
    }

    @Override
    public double getDistanceLeftInches() {
        return encoderL.getRaw() * TICKS_TO_INCHES;
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
    public double getDistanceRightInches() {
        return encoderR.getRaw() * TICKS_TO_INCHES;
    }

    @Override
    public double getHeadingDegrees() {
        return navx.getYaw();
    }

    @Override
    public void stopEverything() {

    }

    @Override
    public void resetArmPosition() {

    }
    @Override
    public void resetClimberPosition() {

    }

    @Override
    public void resetElevatorPosition() {

    }

    @Override
        public void resetDriveEncoders() {
        encoderL.reset();
        encoderR.reset();
    }

    @Override
    public void resetYaw() {
        navx.reset();
    }


    @Override
    public void setRollerInternal(double power) {
        rollLeft .set(ControlMode.PercentOutput, power);
        rollRight.set(ControlMode.PercentOutput, power);
    }

    @Override
    public void checkSafety() {
        if (isElevatorUp()) driveElevator(0);
        if (isElevatorDown()) driveElevator(0);
        if (isArmUp()) driveArm(0);
        if (isArmDown()) driveArm(0);
    }

    @Override
    public double getElevatorEncoderCountInternal() {
        return encoderElevator.get();
    }

    @Override
    public double getArmEncoderCountInternal() {
        return encoderWrist.get();
    }

    @Override
    public double getPitchDegrees() {
        return navx.getPitch();
    }


    @Override
    public double getRollDegrees() {
        return navx.getRoll();
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
    public  void climbLDown(double power){}
    public  void climbRDown(double power){}

    @Override
    public void linearSlideInternal(DoubleSolenoid.Value state) {

    }

    @Override
    public void getClimberCurrent() {

    }

    public  void climb2(boolean state){}
}
