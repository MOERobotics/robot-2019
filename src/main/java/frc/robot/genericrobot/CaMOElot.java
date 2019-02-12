package frc.robot.genericrobot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.*;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.genericrobot.GenericRobot;

public class CaMOElot extends GenericRobot {

    final double TICKS_TO_INCHES = 112.08;
    final double TICKS_TO_FEET = TICKS_TO_INCHES * 12;

    double fakeArmEncoder = 0;
    double fakeElevatorEncoder = 0;
    double fakeTurretEncoder = 0;

    double fakeArmPower = 0;
    double fakeElevatorPower = 0;
    double fakeTurretPower = 0;

    private TalonSRX leftMotorA = new TalonSRX(12);
    private TalonSRX leftMotorB = new TalonSRX(13);
    private TalonSRX leftMotorC = new TalonSRX(14);

    private TalonSRX rightMotorA = new TalonSRX(1);
    private TalonSRX rightMotorB = new TalonSRX(2);
    private TalonSRX rightMotorC = new TalonSRX(3);

    private TalonSRX collector = new TalonSRX(0);

    private TalonSRX shootMotorA = new TalonSRX(10);
    private TalonSRX shootMotorB = new TalonSRX(11);

    Encoder  encoderL    = new Encoder(0, 1, false, CounterBase.EncodingType.k1X);
    Encoder  encoderR    = new Encoder(2, 3, false, CounterBase.EncodingType.k1X);

    AHRS navx = new AHRS(SPI.Port.kMXP,(byte) 50);

    {
        leftMotorA.setNeutralMode(NeutralMode.Brake);
        leftMotorB.setNeutralMode(NeutralMode.Brake);
        leftMotorC.setNeutralMode(NeutralMode.Brake);
        rightMotorA.setNeutralMode(NeutralMode.Brake);
        rightMotorB.setNeutralMode(NeutralMode.Brake);
        rightMotorC.setNeutralMode(NeutralMode.Brake);
        rightMotorA.setInverted(true);
        rightMotorB.setInverted(true);
        rightMotorC.setInverted(true);
    }


    @Override
    public void setDrivePowerInternal(double leftMotor, double rightMotor) {
        //must cap the power to <=1.0 or >= -1.0
        if(leftMotor > 1.0)
            leftMotor = 1.0;
        if(leftMotor < -1.0)
            leftMotor = -1.0;
        if(rightMotor > 1.0)
            rightMotor = 1.0;
        if(rightMotor < -1.0)
            rightMotor = -1.0;

        leftMotorA.set(ControlMode.PercentOutput, leftMotor);
        leftMotorB.set(ControlMode.PercentOutput, leftMotor);
        leftMotorC.set(ControlMode.PercentOutput, leftMotor);

        rightMotorA.set(ControlMode.PercentOutput, rightMotor);
        rightMotorB.set(ControlMode.PercentOutput, rightMotor);
        rightMotorC.set(ControlMode.PercentOutput, rightMotor);
    }

    @Override
    protected void setElevatorInternal(double power) {
        fakeElevatorPower = power;
    }

    @Override
    protected void setTurretInternal(double power) {
        fakeTurretPower = power;
    }

    @Override
    protected void setArmInternal(double power) {
        fakeArmPower = 0;
    }

    @Override
    public double getDistanceLeftInches() {
        return encoderL.getRaw() / TICKS_TO_INCHES;
    }

    @Override
    public double getDistanceRightInches() {
        return encoderL.getRaw() / TICKS_TO_INCHES;
    }

    @Override
    public double getHeadingDegrees() {
        return navx.getYaw();
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
    public void stopEverything() {
        setDrivePower(0,0);
    }

    @Override
    public void stopDriving() {
    setDrivePower(0,0);
    }

    @Override
    public void setRollerInternal(double power) {
        shootMotorA.set(ControlMode.PercentOutput,power*0.2);
        shootMotorB.set(ControlMode.PercentOutput,power*0.2);
    }


    @Override
    public void grabHatch() {

    }

    @Override
    public void checkSafety() {
        fakeTurretEncoder   += fakeTurretPower;
        fakeArmEncoder      += fakeArmPower;
        fakeElevatorEncoder += fakeElevatorPower;

    }

    @Override
    public void driveSA(double power) {

    }

    @Override
    public void driveSB(double power) {

    }

    @Override
    public void driveFA(double power) {

    }

    @Override
    public void driveFB(double power) {

    }

    @Override
    public double getElevatorEncoderCount() {
        return fakeElevatorEncoder;
    }

    @Override
    public double getTurretEncoderCount() {
        return fakeTurretEncoder;
    }

    @Override
    public double getArmEncoderCount() {
        return fakeArmEncoder;
    }

}