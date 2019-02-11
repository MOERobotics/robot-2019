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

    final double TICKS_TO_INCHES = 45;
    final double TICKS_TO_FEET = TICKS_TO_INCHES * 12;

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
    public double getDistanceLeftInches() {
        return encoderL.getRaw() / TICKS_TO_INCHES;
    }

    @Override
    public double getDistanceRightInches() {
        return encoderR.getRaw() / TICKS_TO_INCHES;
    }

    @Override
    public double getHeadingDegrees() {
        return navx.getYaw();
    }

    @Override
    public void stopEverything() {
        setDrivePower(0,0);
    }

    @Override
        public void resetDriveEncoder() {
        encoderL.reset();
        encoderR.reset();
    }

    @Override
    public void resetYaw() {
        navx.reset();
    }

    @Override
    public void stopDriving() {
        setDrivePower(0,0);
    }

    @Override
    public double getPitch() {
        return navx.getPitch();
    }

    @Override
    public double getRolling() {
        return navx.getRoll();
    }
}
