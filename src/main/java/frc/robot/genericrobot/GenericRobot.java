package frc.robot.genericrobot;

/*code*/

import edu.wpi.first.wpilibj.DoubleSolenoid;

public abstract class GenericRobot {

	//Last recorded values
    private double leftPower;
    private double rightPower;
    private double elevatorPower;
    private double turretPower;
    private double armPower;
    private double rollerPower;

	//checking for things
	public abstract double getDistanceLeftInches();
	public abstract double getDistanceRightInches();
	public abstract double getHeadingDegrees();
	public abstract double getPitchDegrees();
	public abstract double getRollDegrees();
	public abstract double getElevatorEncoderCount();
	public abstract double getTurretEncoderCount();
	public abstract double getArmEncoderCount();

	//stopping and resetting
	public abstract void resetDriveEncoders();
	public abstract void resetYaw();
	public abstract void checkSafety();
	public void stopEverything() {
		stopDriving();
		driveElevator(0);
		driveArm(0);
		driveTurret(0);
	}
	public void stopDriving() {
		setDrivePower(0,0);
	}

    //Drive <editor-fold>
    public final void   moveForward        (double motorPower) { setDrivePower( motorPower,  motorPower); }
    public final void   moveBackward       (double motorPower) { setDrivePower(-motorPower, -motorPower); }
    public final void   turnLeftInplace    (double motorPower) { setDrivePower(-motorPower,  motorPower); }
    public final void   turnRightInplace   (double motorPower) { setDrivePower( motorPower, -motorPower); }
    public final void   setDrivePower      (double leftMotor, double rightMotor) {
        this. leftPower =  leftMotor;
        this.rightPower = rightMotor;
        setDrivePowerInternal(leftMotor, rightMotor);
    }
	public final double getLeftDrivePower  () { return  leftPower; }
	public final double getRightDrivePower () { return rightPower; }
	protected abstract void setDrivePowerInternal(double leftMotor, double rightMotor);
	//</editor-fold>

    //Elevator <editor-fold>
    public final void    elevatorUp       (double power) {driveElevator( power);}
    public final void    elevatorDown     (double power) {driveElevator(-power);}
	public final void    driveElevator    (double power) {
		this.elevatorPower = power;
		if      (isElevatorUp  () && power > 0) setElevatorInternal(  0.0);
		else if (isElevatorDown() && power < 0) setElevatorInternal(  0.0);
		else                                    setElevatorInternal(power);
	}
	public       boolean isElevatorUp     () {return         false;}
	public       boolean isElevatorDown   () {return         false;}
	public final double  getElevatorPower () {return elevatorPower;}
    protected abstract void setElevatorInternal(double power);
	//</editor-fold>

    //Turret <editor-fold>
    public final void    turretLeft     (double power) {driveTurret( power);}
    public final void    turretRight    (double power) {driveTurret(-power);}
    public final void    driveTurret    (double power) {
        this.turretPower = power;
        if      (isTurretRight() && power > 0)  setTurretInternal(  0.0);
        else if (isTurretLeft () && power < 0)  setTurretInternal(  0.0);
        else                                    setTurretInternal(power);
    }
	public       boolean isTurretRight  () {return       false;}
	public       boolean isTurretLeft   () {return       false;}
	public final double  getTurretPower () {return turretPower;}
	protected abstract void setTurretInternal(double power);
	//</editor-fold>

    //Arm <editor-fold>
    public final void    driveArm    (double power)  {
        this.armPower = power;
        if      (isArmUp  () && power > 0) setArmInternal(  0.0);
        else if (isArmDown() && power < 0) setArmInternal(  0.0);
        else                               setArmInternal(power);
    }
	public       boolean isArmUp     () {return    false;}
	public       boolean isArmDown   () {return    false;}
	public final double  getArmPower () {return armPower;}
	protected abstract void setArmInternal(double power);
	//</editor-fold>

    //Roller <editor-fold>
	public final void   rollIn      (double power) { driveRoller(-power); }
	public final void   rollOut     (double power) { driveRoller( power); }
	public final void   driveRoller (double power) {
		this.rollerPower = power;
		setRollerInternal(power);
	}
	public final double getRollerPower () {return rollerPower;}
	protected abstract void setRollerInternal(double power);
	//</editor-fold>

    //Temporary for SuperMOEva testing
    public void driveSA(double power) {};
    public void driveSB(double power) {};
    public void driveFA(double power) {};
    public void driveFB(double power) {};

    public abstract void shiftHigh();
    public abstract void shiftLow();
    public void shiftDrive(DoubleSolenoid.Value value) {
    	//this.shifterSolenoidValue = value;
    	//shiftDriveInternal(value);
	}

	/*public void shiftDriveInternal(DoubleSolenoid shifter, DoubleSolenoid.Value value) {
		shifter.set(value);
    }*/

	//needs work
	public void grabHatch() {

	}

}

