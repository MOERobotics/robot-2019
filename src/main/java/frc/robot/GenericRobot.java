package frc.robot;
/*code*/

public abstract class GenericRobot {
    public abstract void moveForward(double motorPower);
    public abstract void moveBackward(double motorPower);
    public abstract void setDrivePower(double leftMotor, double rightMotor);
    public abstract void turnLeft(double motorPower);
    public abstract void turnRight(double motorPower);
    /*Two types of turning, still and hooked. Add hooked later*/
    public abstract double getDistanceLeftInches();
    public abstract double getDistanceRightInches();
    public abstract double getHeadingDegrees();
    public abstract void resetDistance();
    public abstract void resetDegrees();
    public abstract void stopEverything();
    public abstract void stopMotors();
}

