package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;

public class MOErio extends GenericRobot{
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

    private TalonSRX leftMotorA = new TalonSRX(12);
    private TalonSRX leftMotorB = new TalonSRX(13);
    private TalonSRX leftMotorC = new TalonSRX(14);

    private TalonSRX rightMotorA = new TalonSRX(1);
    private TalonSRX rightMotorB = new TalonSRX(2);
    private TalonSRX rightMotorC = new TalonSRX(3);

    private TalonSRX collector = new TalonSRX(0);

    private TalonSRX shootMotorA = new TalonSRX(10);
    private TalonSRX shootMotorB = new TalonSRX(11);

    {
        driveRA.setInverted(true);
        driveRB.setInverted(true);
        rollLeft.setInverted(true);
    }

    @Override
    public void setDrivePower(double leftMotor, double rightMotor) {
        leftMotorA.set(ControlMode.PercentOutput, leftMotor);
        leftMotorB.set(ControlMode.PercentOutput, leftMotor);
        leftMotorC.set(ControlMode.PercentOutput, leftMotor);

        rightMotorA.set(ControlMode.PercentOutput, rightMotor);
        rightMotorB.set(ControlMode.PercentOutput, rightMotor);
        rightMotorC.set(ControlMode.PercentOutput, rightMotor);
    }

    @Override
    public void moveForward(double motorPower) {
        setDrivePower(motorPower,motorPower);
    }

    @Override
    public void moveBackward(double motorPower) {
        setDrivePower(-motorPower,-motorPower);
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
    public void stopEverything() {

    }

    @Override
        public void resetDistance() {

    }

    @Override
    public void resetDegrees() {

    }

    @Override
    public void stopMotors() {

    }
}
