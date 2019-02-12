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
    TalonSRX driveLA   = new TalonSRX( 0) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveLB   = new TalonSRX(15) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveRA   = new TalonSRX( 1) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveRB   = new TalonSRX(14) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX elevator  = new TalonSRX( 2) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX rollLeft  = new TalonSRX(12) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX rollRight = new TalonSRX( 3) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX wrist     = new TalonSRX( 4) {{setNeutralMode(NeutralMode.Brake);}};

    Encoder encoderL        = new Encoder(0, 1, false, CounterBase.EncodingType.k1X);
    Encoder encoderR        = new Encoder(2, 3, false, CounterBase.EncodingType.k1X);
    Encoder encoderElevator = new Encoder(4, 5, true,  CounterBase.EncodingType.k2X);
    Encoder encoderWrist    = new Encoder(8, 9, true,  CounterBase.EncodingType.k2X);

    DigitalInput elevatorBottomLimitSwitch = new DigitalInput(6);
    DigitalInput elevatorTopLimitSwitch    = new DigitalInput(7);

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
    protected void setElevatorInternal(double power) {
        elevator.set(ControlMode.PercentOutput, power);
    }

    @Override
    protected void setTurretInternal(double power) {
        //no turret!
    }

    @Override
    protected void setArmInternal(double power) {
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
    public void setRollerInternal(double power) {
        rollLeft .set(ControlMode.PercentOutput, power);
        rollRight.set(ControlMode.PercentOutput, power);
    }


    @Override
    public void grabHatch() {

    }

    @Override
    public void checkSafety() {
        if (isElevatorUp()) driveElevator(0);
        if (isElevatorDown()) driveElevator(0);
        if (isArmUp()) driveArm(0);
        if (isArmDown()) driveArm(0);
    }


    @Override
    public double getElevatorEncoderCount() {
        return encoderElevator.get();
    }

    @Override
    public double getTurretEncoderCount() {
        return 0;
    }

    @Override
    public double getArmEncoderCount() {
        return encoderWrist.get();
    }

    @Override
    public boolean isElevatorUp() {
        return elevatorTopLimitSwitch.get();
    }

    @Override
    public boolean isElevatorDown() {
        return elevatorBottomLimitSwitch.get();
    }

    @Override
    public boolean isArmUp() {
        return encoderWrist.getRaw() < 0;
    }

    @Override
    public boolean isArmDown() {
        return encoderWrist.getRaw() > 1150;
    }
}
