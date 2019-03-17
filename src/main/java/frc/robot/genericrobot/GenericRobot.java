package frc.robot.genericrobot;

/*code*/

import edu.wpi.first.wpilibj.DoubleSolenoid;

public abstract class GenericRobot {

	//Last recorded values
    private double        leftPower;
    private double       rightPower;
    private double    elevatorPower;
    private double      turretPower;
    private double         armPower;
    private double      rollerPower;
	private double       climbPower;
    private boolean spearShaftState;
    private boolean  spearHookState;
    private boolean floorPickupState;
	private boolean shifterSolenoidState;
	private DoubleSolenoid.Value climbPushForwardzState;
	private DoubleSolenoid.Value climb2State;

    private double       armEncoderOffset = 0;
    //private double    turretEncoderOffset = 0;
    private double  elevatorEncoderOffset = 0;
    private boolean   totalSafetyOverride = false;

    /*
    public abstract double getElevatorEncoderStart();
    public abstract double getArmEncoderStart();
	*/

    //find origin
    /*private double elevatorOrigin;
    public void setElevatorOrigin(double origin) {
    	elevatorOrigin = origin;
	}
	public double getElevatorOrigin() {
    	return elevatorOrigin;
	}

	private double armOrigin;
	public void setArmOrigin(double origin) {
		armOrigin = origin;
	}
	public double getArmOrigin() {
		return armOrigin;
	}*/

    //lidar
	public abstract int numSensors();
	public int[] lidar = new int[numSensors()];

	//checking for things
	public abstract double getDistanceLeftInches();
	public abstract double getDistanceRightInches();
	public abstract double getHeadingDegrees();
	public abstract double getPitchDegrees();
	public abstract double getRollDegrees();
	public abstract double getElevatorEncoderCountInternal();
	//public abstract double getTurretEncoderCountInternal();
	public abstract double getArmEncoderCountInternal();

	//stopping and resetting
	public abstract void resetDriveEncoders();
	public abstract void resetYaw();
	public abstract void checkSafety();
	public void stopEverything() {
		stopDriving();
		driveElevator(0);
		driveArm(0);
		//driveTurret(0);
	}
	public void stopDriving() {
		setDrivePower(0,0);
	}
	public abstract void resetArmPosition();
	public abstract void resetElevatorPosition();
	public abstract void resetClimberPosition();

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

	//shifting
	public void shiftLow () { shiftDrive(true); }
	public void shiftHigh  () { shiftDrive(false); }

	public void shiftDrive(boolean state) {
		this.shifterSolenoidState = state;
		shiftDriveInternal(state);
	}
	public abstract void shiftDriveInternal(boolean value);
	public boolean getShifterSolenoidState() { return shifterSolenoidState; }

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
	public       void    enableElevatorLimits(boolean enabled) {}
    protected abstract void setElevatorInternal(double power);
	public boolean isElevForwardLimitEnabled() {return false;}
	public boolean isElevReverseLimitEnabled() {return false;}
	/*public boolean atElevForwardLimit() {return false;}
	public boolean atElevReverseLimit() {return false;}*/

	//</editor-fold>

    //Turret <editor-fold>
    /*public final void    turretLeft     (double power) {driveTurret( power);}
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
	protected abstract void setTurretInternal(double power);*/
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
    public void    enableArmLimits(boolean enabled) {}
    public boolean isArmForwardLimitEnabled() {return false;}
    public boolean isArmReverseLimitEnabled() {return false;}
    /*public boolean atArmForwardLimit() {return false;}
    public boolean atArmReverseLimit() {return false;}*/
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

	//Hatch Grab <editor-fold>
	public void spearIn()  { shiftSpearShaft(false); }
	public void spearOut() { shiftSpearShaft(true); }
	public void shiftSpearShaft(boolean out) {
		this.spearShaftState = out;
		shiftSpearShaftInternal(out);
	}
	public abstract void shiftSpearShaftInternal(boolean out);
	public boolean getSpearShaftState() {return spearShaftState;}

	public void spearHook  () { shiftSpearHook(false); }
	public void spearUnhook() { shiftSpearHook( true); }
	public void shiftSpearHook(boolean out) {
		this.spearHookState = out;
		shiftSpearHookInternal(out);
	}
	public abstract void shiftSpearHookInternal(boolean out);
	public boolean getSpearHookState() {return spearHookState;}
	//</editor-fold>

	//combos
	//public abstract void grabberOpenCombo();
	//public abstract void grabberClosedCombo();

	//Floor Hatch Grab
	public void floorPickupUp() { shiftFloorPickup(false); }
	public void floorPickupDown() { shiftFloorPickup(true); }
	public void shiftFloorPickup(boolean out) {
		this.floorPickupState = out;
		shiftFloorPickupInternal(out);
	}
	public void shiftFloorPickupInternal(boolean out) {};
	public boolean getFloorPickupState() {return floorPickupState;}

	//Habitat Climb <editor-fold>
	//public void climbUp  (double power) {climb(-power);}
	//public void climbDown(double power) {climb( power);}
	public void climb(double power) {
		this.climbPower = power;
		climbInternal(power);
	}
    public abstract void climbInternal(double power);
    public abstract double getClimberLEncoderCount();
    public abstract double getClimberREncoderCount();
    public double getClimbPower() {return this.climbPower;}

    public abstract void climbLDown(double power);
    public abstract void climbRDown(double power);
    public abstract void climb2Internal(DoubleSolenoid.Value state);
    public void LinearSlider(DoubleSolenoid.Value value) {
        climb2Internal(value);
        this.climb2State = value;
    }
    public DoubleSolenoid.Value getClimb2State() {
        return climb2State;
    }

    public abstract void climbPushForwardzInternal(DoubleSolenoid.Value value);
    public void climbPushForwardz(DoubleSolenoid.Value value) {
        climbPushForwardzInternal(value);
        this.climbPushForwardzState = value;
    }
    public DoubleSolenoid.Value getClimbPushForwardzState() {
        return climbPushForwardzState;
    }

    public abstract void footSpacerCylinderInternal(DoubleSolenoid.Value value);
    public void footSpacerCylinder(DoubleSolenoid.Value value){
    	footSpacerCylinderInternal(value);
	}

    //Temporary for SuperMOEva testing
    /*public void driveSA(double power) {};
    public void driveSB(double power) {};
    public void driveFA(double power) {};
    public void driveFB(double power) {};*/

    public void setOffsets() {
    	this.armEncoderOffset = getArmEncoderCountInternal();
    	this.elevatorEncoderOffset = getElevatorEncoderCountInternal();
    	//this.turretEncoderOffset = getTurretEncoderCountInternal();
	}

	public void clearOffsets() {
    	this.armEncoderOffset = 0;
    	this.elevatorEncoderOffset = 0;
    	//this.turretEncoderOffset = 0;
	}

	public void setSafetyOverride(boolean state) {
    	totalSafetyOverride = state;
	}

	public boolean getSafetyOverride() {return totalSafetyOverride;}

	public double getArmEncoderCount() {
    	return getArmEncoderCountInternal() - armEncoderOffset;
	}
	public double getElevatorEncoderCount() {
		return getElevatorEncoderCountInternal() - elevatorEncoderOffset;
	}
	/*public double getTurretEncoderCount() {
		return getTurretEncoderCountInternal() - turretEncoderOffset;
	}*/

	public abstract boolean getSix();
	public abstract boolean getSeven();

}

