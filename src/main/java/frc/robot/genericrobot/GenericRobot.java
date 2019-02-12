package frc.robot.genericrobot;

/*code*/

public abstract class GenericRobot {
    //movement of robot

    private double leftPower;
    private double rightPower;
    private double elevatorPower;
    private double turretPower;
    private double armPower;
    private double rollerPower;

    public void moveForward(double motorPower) {
        setDrivePower(motorPower, motorPower);
    }
    public void moveBackward(double motorPower) {
        setDrivePower(-motorPower, -motorPower);
    }
    public void turnLeftInplace(double motorPower) {
        setDrivePower(-motorPower, motorPower);
    }
    public void turnRightInplace(double motorPower) { setDrivePower(motorPower, -motorPower);    }
    public double getLeftDrivePower () { return  leftPower; }
    public double getRightDrivePower() { return rightPower; }
    protected abstract void setDrivePowerInternal(double leftMotor, double rightMotor);
    public final void setDrivePower(double leftMotor, double rightMotor) {
        leftPower  =  leftMotor;
        rightPower = rightMotor;
        setDrivePowerInternal(leftMotor, rightMotor);
    }

   //checking for things
    public abstract double getDistanceLeftInches();
    public abstract double getDistanceRightInches();
    public abstract double getHeadingDegrees();


    //stopping and resetting
    public abstract void resetDriveEncoders();
    public abstract void resetYaw();
    public abstract void stopEverything();
    public abstract void stopDriving();

    //moving the elevator
    public boolean isElevatorUp() {return false;}
    public boolean isElevatorDown() {return false;}
    public void elevatorUp(double power)   {driveElevator( power);}
    public void elevatorDown(double power) {driveElevator(-power);}
    public double getElevatorPower() {return elevatorPower;}
    protected abstract void setElevatorInternal(double power);
    public final void driveElevator(double power) {
        elevatorPower = power;
        if (isElevatorUp() && power > 0) {
            setElevatorInternal(0);
        } else if (isElevatorDown() && power < 0) {
            setElevatorInternal(0);
        } else {
            setElevatorInternal(power);
        }
    }
    public abstract double getElevatorEncoderCount();

    public boolean isTurretRight() {return false;}
    public boolean isTurretLeft() {return false;}
    public void turretLeft (double power) {driveTurret( power);}
    public void turretRight(double power) {driveTurret(-power);}
    public double getTurretPower()   {return turretPower;}
    protected abstract void setTurretInternal(double power);
    public final void driveTurret(double power) {
        turretPower = power;
        if (isTurretRight() && power > 0) {
            setTurretInternal(0);
        } else if (isTurretLeft() && power < 0) {
            setTurretInternal(0);
        } else {
            setTurretInternal(power);
        }
    }
    public abstract double getTurretEncoderCount();

    public boolean isArmUp  () {return false;}
    public boolean isArmDown() {return false;}
    public double getArmPower()      {return armPower;}
    protected abstract void setArmInternal(double power);
    public final void driveArm(double power) {
        armPower = power;
        if (isArmUp() && power > 0) {
            setArmInternal(0);
        } else if (isArmDown() && power < 0) {
            setArmInternal(0);
        } else {
            setArmInternal(power);
        }
    }
    public abstract double getArmEncoderCount();

    public abstract void grabHatch();

    public abstract void checkSafety();

    //moving the wrist


    //moving the collector

    //just for now
    public abstract void driveSA(double power);
    public abstract void driveSB(double power);
    public abstract void driveFA(double power);
    public abstract void driveFB(double power);

    public double getRollerPower()   {return rollerPower;}
    public void driveRoller(double power) {
        this.rollerPower = power;
        setRollerInternal(power);
    }
    protected abstract void setRollerInternal(double power);
    public void rollIn (double power) { driveRoller(-power); }
    public void rollOut(double power) {
        driveRoller( power);
    }


}

