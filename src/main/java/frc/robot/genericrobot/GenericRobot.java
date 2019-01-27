package frc.robot.genericrobot;

/*code*/

public abstract class GenericRobot {
    //movement of robot

    double leftPower;
    double rightPower;

    public void moveForward(double motorPower) {
        setDrivePower(motorPower, motorPower);
    }
    public void moveBackward(double motorPower) {
        setDrivePower(-motorPower, -motorPower);
    }
    public void turnLeftInplace(double motorPower) {
        setDrivePower(-motorPower, motorPower);
    }
    public void turnRightInplace(double motorPower) {
        setDrivePower(motorPower, -motorPower);
    }
    abstract void setDrivePowerInternal(double leftMotor, double rightMotor);
    public void setDrivePower(double leftMotor, double rightMotor) {
        leftPower = leftMotor;
        rightPower = rightMotor;
        setDrivePowerInternal(leftMotor, rightMotor);
    }

   //checking for things
    public abstract double getDistanceLeftInches();
    public abstract double getDistanceRightInches();
    public abstract double getHeadingDegrees();

    public double getLeftDrivePower () { return leftPower; }
    public double getRightDrivePower() { return rightPower; }

    //stopping and resetting
    public abstract void resetDriveEncoder();
    public abstract void resetYaw();
    public abstract void stopEverything();
    public abstract void stopDriving();

    //moving the elevator
    //public abstract void elevatorUp();
    //public abstract void elevatorDown();
    //public abstract void elevatorTurnLeft();
    //public abstract void elevatorTurnRight();

    //moving the wrist


    //moving the collector
}

