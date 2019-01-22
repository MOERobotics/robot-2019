package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;

public class CaMOElot extends GenericRobot {

    final double TICKS_TO_INCHES = 112.08;
    final double TICKS_TO_FEET = TICKS_TO_INCHES * 12;

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

    public AHRS navx = new AHRS(SPI.Port.kMXP,(byte) 50);

    {
        rightMotorA.setInverted(true);
        rightMotorB.setInverted(true);
        rightMotorC.setInverted(true);
    }

    @Override
    public double getAccelX(){
        return navx.getWorldLinearAccelX();
    }

    @Override
    public void setDrivePower(double leftMotor, double rightMotor) {
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
    public void moveForward(double motorPower) {
        setDrivePower(motorPower, motorPower);
    }

    @Override
    public void moveBackward(double motorPower) {
        setDrivePower(-motorPower, -motorPower);
    }

    @Override
    public void turnLeft(double motorPower) {
    setDrivePower(-motorPower, motorPower);
    }

    @Override
    public void turnRight(double motorPower) {
    setDrivePower(motorPower, -motorPower);
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
    public void resetDistance() {
        encoderL.reset();
    }

    @Override
    public void resetDegrees() {
        navx.reset();
    }


    @Override
    public void stopEverything() {
        setDrivePower(0,0);
    }

    @Override
    public void stopMotors() {
    setDrivePower(0,0);
    }
}