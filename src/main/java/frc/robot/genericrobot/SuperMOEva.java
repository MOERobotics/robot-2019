package frc.robot.genericrobot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.*;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.genericrobot.GenericRobot;

public class SuperMOEva extends GenericRobot{

    final double TICKS_TO_INCHES = 45;
    final double TICKS_TO_FEET = TICKS_TO_INCHES * 12;

    Encoder encoderL = new Encoder(0, 1, false, CounterBase.EncodingType.k1X);
    Encoder encoderR = new Encoder(2, 3, false, CounterBase.EncodingType.k1X);

    AHRS navx = new AHRS(SPI.Port.kMXP,(byte) 50);

    @Override
    void setDrivePowerInternal(double leftMotor, double rightMotor) {
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
    public void resetDriveEncoder() {
        encoderL.reset();
        encoderR.reset();
    }

    @Override
    public void resetYaw() {
        navx.reset();
    }

    @Override
    public void stopEverything() {
        stopDriving();
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
