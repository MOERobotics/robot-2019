package frc.robot;
/*code*/

public abstract class GenericRobot {
    public abstract void moveForward(double motorPower);
    public abstract void moveBackward(double motorPower);
    public abstract void setDrivePower(double leftMotor, double rightMotor);
    public abstract void turnLeft(double degrees);
    public abstract void turnRight(double degrees);
    /*Two types of turning, still and hooked. Add hooked later*/
    public abstract double getDistance();
    public abstract double getDegrees();
    public abstract void stopEverything();
    public abstract void stopMotors();
}

