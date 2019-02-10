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
    TalonSRX driveLA = new TalonSRX(0) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveLB = new TalonSRX(15) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveRA = new TalonSRX(1) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveRB = new TalonSRX(14) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX elevator = new TalonSRX(2) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX rollLeft = new TalonSRX(12) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX rollRight = new TalonSRX(3) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX wrist = new TalonSRX(4) {{setNeutralMode(NeutralMode.Brake);}};

    Encoder encoderL = new Encoder(0, 1, false, CounterBase.EncodingType.k1X);
    Encoder encoderR = new Encoder(2, 3, false, CounterBase.EncodingType.k1X);

    AHRS navx = new AHRS(SPI.Port.kMXP,(byte) 50);

    {
        driveRA.setInverted(true);
        driveRB.setInverted(true);
        rollLeft.setInverted(true);
    }

    @Override
    public void setDrivePowerInternal(double leftMotor, double rightMotor) {
        driveLA.set(ControlMode.PercentOutput, leftMotor);
        driveLB.set(ControlMode.PercentOutput, leftMotor);

        driveRA.set(ControlMode.PercentOutput, rightMotor);
        driveRB.set(ControlMode.PercentOutput, rightMotor);
    }

    @Override
    void setElevatorInternal(double power) {
        elevator.set(ControlMode.PercentOutput, power);
    }

    @Override
    void setTurretInternal(double power) {
        //no turret!
    }

    @Override
    void setArmInternal(double power) {
        wrist.set(ControlMode.PercentOutput, power);
    }

    @Override
    public double getDistanceLeftInches() {
        return encoderL.getRaw();
    }

    @Override
    public double getDistanceRightInches() {
        return encoderR.getRaw();
    }

    @Override
    public double getHeadingDegrees() {
        return navx.getYaw();
    }

    @Override
    public void stopEverything() {

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
    public void stopDriving() {

    }

    @Override
    public void driveRoll(double power) {

    }

    @Override
    public void rollIn() {

    }

    @Override
    public void rollOut() {

    }

    @Override
    public void grabHatch() {

    }

    @Override
    public void checkSafety() {

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
}
