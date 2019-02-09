package frc.robot.genericrobot;

/*code*/

public abstract class GenericRobot {
    //movement of robot

    double leftPower;
    double rightPower;
    double elevatorPower;
    double turretPower;
    double armPower;

    //public static int numSensors = 2;
    //public static int[] lidar = new int[numSensors];

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
    abstract void setElevatorInternal(double power);
    abstract void setTurretInternal(double power);
    abstract void setArmInternal(double power);
    public final void setDrivePower(double leftMotor, double rightMotor) {
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
    public abstract void resetDriveEncoders();
    public abstract void resetYaw();
    public abstract void stopEverything();
    public abstract void stopDriving();

    //moving the elevator
    public void elevatorUp(double power) {driveElevator(1);}
    public void elevatorDown(double power) {driveElevator(-1);}
    public void turretLeft(double power) {driveTurret(1);}
    public void turretRight(double power){driveTurret(-1);}

    public final void driveElevator(double power) {
        elevatorPower = power;
        if (isElevatorUp() && power > 0) {
            setElevatorInternal(power);
        } else if (isElevatorUp() && power < 0) {
            setElevatorInternal(power);
        } else {
            setElevatorInternal(0);
        }
    }

    public final void driveTurret(double power) {
        turretPower = power;
        if (isTurretRight() && power > 0) {
            setTurretInternal(power);
        } else if (isTurretLeft() && power < 0) {
            setTurretInternal(power);
        } else {
            setTurretInternal(0);
        }
    }

    public final void driveArm(double power) {
        armPower = power;
        if (isArmUp() && power > 0) {
            setArmInternal(power);
        } else if (isArmDown() && power < 0) {
            setArmInternal(power);
        } else {
            setArmInternal(0);
        }
    }

    public boolean isElevatorUp() {return false;}
    public boolean isElevatorDown() {return false;}

    public double getElevatorPower() {return elevatorPower;}

    public boolean isTurretRight() {return false;}
    public boolean isTurretLeft() {return false;}

    public double getTurretPower() {return turretPower;}

    public boolean isArmUp() {return false;}
    public boolean isArmDown() {return false;}

    public double getArmPower() {return armPower;}

    public abstract void checkSafety();

    //moving the wrist


    //moving the collector
}

