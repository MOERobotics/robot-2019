package frc.robot;

public class CaMOElot extends GenericRobot {

    Joystick leftJoystick = new Joystick(0);


    TalonSRX leftMotorA = new TalonSRX(12);
    TalonSRX leftMotorB = new TalonSRX(13);
    TalonSRX leftMotorC = new TalonSRX(14);

    TalonSRX rightMotorA = new TalonSRX(1);
    TalonSRX rightMotorB = new TalonSRX(2);
    TalonSRX rightMotorC = new TalonSRX(3);

    TalonSRX collector = new TalonSRX(0);

    TalonSRX shootMotorA = new TalonSRX(10);
    TalonSRX shootMotorB = new TalonSRX(11);
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
    public double getDistance() {
        return 0;
    }

    @Override
    public double getDegrees() {
        return 0;
    }

    @Override
    public void stopEverything() {
        setDrivePower(0,0);
    }

    @Override
    public void stopMotors() {
        setDrivePower(0,0);
    }