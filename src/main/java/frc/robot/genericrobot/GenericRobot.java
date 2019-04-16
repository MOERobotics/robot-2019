package frc.robot.genericrobot;

/*code*/

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Logger;
import frc.robot.PixyCam;
import io.github.pseudoresonance.pixy2api.Pixy2Line;
import static frc.robot.Logger.doubleToPercent;

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
	private boolean spacerState;
	//private DoubleSolenoid.Value climbPushForwardzState;
	private DoubleSolenoid.Value linearSlideState;

    private double       armEncoderOffset = 0;
    //private double    turretEncoderOffset = 0;
    private double  elevatorEncoderOffset = 0;
    private boolean   totalSafetyOverride = false;

    //lidar
	public abstract int numSensors();
	public int[] lidar = new int[numSensors()];
	public long lidarReadTime;
	/*public int leftLidar = lidar[0];
	public int centerLidar = lidar[2];
	public int rightLidar = lidar[1];*/


	//pi
    public int[] piXY = new int[2];

	//pixy
	public PixyCam pixy = new PixyCam() {{
		init();
		run();
		start();
	}};

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
	public Logger<Integer> leftDriverLogger = new Logger<>();
	public Logger<Integer> rightDriverLogger = new Logger<>();
    public final void   moveForward        (double motorPower) { setDrivePower( motorPower,  motorPower); }
    public final void   moveBackward       (double motorPower) { setDrivePower(-motorPower, -motorPower); }
    public final void   turnLeftInplace    (double motorPower) { setDrivePower(-motorPower,  motorPower); }
    public final void   turnRightInplace   (double motorPower) { setDrivePower( motorPower, -motorPower); }
    public final void   setDrivePower      (double leftMotor, double rightMotor) {
    	leftDriverLogger.printIfChanged("Left Driver Motor", doubleToPercent(leftMotor));
    	rightDriverLogger.printIfChanged("Right Driver Motor", doubleToPercent(rightMotor));
        this. leftPower =  leftMotor;
        this.rightPower = rightMotor;
        setDrivePowerInternal(leftMotor, rightMotor);
    }
	public final double getLeftDrivePower  () { return  leftPower; }
	public final double getRightDrivePower () { return rightPower; }
	protected abstract void setDrivePowerInternal(double leftMotor, double rightMotor);

	//shifting
	public Logger<Boolean> shifterLogger = new Logger<>();
	public void shiftLow () { shiftDrive(false); }
	public void shiftHigh  () { shiftDrive(true); } //check on MOEva

	public void shiftDrive(boolean state) {
		shifterLogger.printIfChanged("Is Shifter High?: ", state);
		this.shifterSolenoidState = state;
		shiftDriveInternal(state);
	}
	public abstract void shiftDriveInternal(boolean value);
	public boolean getShifterSolenoidState() { return shifterSolenoidState; }

	//</editor-fold>

    //Elevator <editor-fold>
	public Logger<Integer> elevatorLogger = new Logger<>();
    public final void    elevatorUp       (double power) {driveElevator( power);}
    public final void    elevatorDown     (double power) {driveElevator(-power);}
	public final void    driveElevator    (double power) {
    	elevatorLogger.printIfChanged("Elevator", doubleToPercent(power));
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
	public abstract boolean atElevatorTopLimit();
	public abstract boolean atElevatorBottomLimit();
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
	public Logger<Integer> armLogger = new Logger<>();
    public final void    driveArm    (double power)  {
		armLogger.printIfChanged("Arm", doubleToPercent(power));
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
	public Logger<Integer> rollerLogger = new Logger<>();
	public final void   rollIn      (double power) { driveRoller(power); }
	public final void   rollOut     (double power) { driveRoller(-power); }
	public final void   driveRoller (double power) {
		rollerLogger.printIfChanged("Roller", doubleToPercent(power));
		this.rollerPower = power;
		setRollerInternal(power);
	}
	public final double getRollerPower () {return rollerPower;}
	protected abstract void setRollerInternal(double power);
	//</editor-fold>

	//Hatch Grab <editor-fold>
	public Logger<Boolean> spearLogger = new Logger<>();
	public void spearIn()  { shiftSpearShaft(false); }
	public void spearOut() { shiftSpearShaft(true); }
	public void shiftSpearShaft(boolean out) {
		spearLogger.printIfChanged("Is Spear Out?: ", out);
		this.spearShaftState = out;
		shiftSpearShaftInternal(out);
	}
	public abstract void shiftSpearShaftInternal(boolean out);
	public boolean getSpearShaftState() {return spearShaftState;}

	public Logger<Boolean> spearHookLogger = new Logger<>();
	public void spearHook  () { shiftSpearHook(false); }
	public void spearUnhook() { shiftSpearHook( true); }
	public void shiftSpearHook(boolean out) {
		spearHookLogger.printIfChanged("Is Spear Unhooked?: ", out);
		this.spearHookState = out;
		shiftSpearHookInternal(out);
	}
	public abstract void shiftSpearHookInternal(boolean out);
	public boolean getSpearHookState() {return spearHookState;}
	//</editor-fold>

	//Floor Hatch Grab
	public Logger<Boolean> floorPickupLogger = new Logger<>();
	public void floorPickupUp() { shiftFloorPickup(false); }
	public void floorPickupDown() { shiftFloorPickup(true); }
	public void shiftFloorPickup(boolean out) {
		floorPickupLogger.printIfChanged("Is Floor Pickup Down?: ", out);
		this.floorPickupState = out;
		shiftFloorPickupInternal(out);
	}
	public void shiftFloorPickupInternal(boolean out) {};
	public boolean getFloorPickupState() {return floorPickupState;}

	//Habitat Climb <editor-fold>
	//public void climbUp  (double power) {climb(-power);}
	//public void climbDown(double power) {climb( power);}
	public Logger<Integer> climberLogger = new Logger<>();
	public void climb(double power) {
		climberLogger.printIfChanged("Climber", doubleToPercent(power));
		this.climbPower = power;
		climbInternal(power);
	}
    public abstract void climbInternal(double power);
    public abstract double getClimberLEncoderCount();
    public abstract double getClimberREncoderCount();
    public double getClimbPower() {return this.climbPower;}

    public abstract void climbLDown(double power);
    public abstract void climbRDown(double power);
    public abstract void linearSlideInternal(DoubleSolenoid.Value state);
    public void LinearSlider(DoubleSolenoid.Value value) {
        linearSlideInternal(value);
        this.linearSlideState = value;
    }
    public DoubleSolenoid.Value getClimb2State() {
        return linearSlideState;
    }
    public abstract void getClimberCurrent();

    public abstract void footSpacerCylinderInternal(boolean state);
    public void footSpacerCylinder(boolean state){
    	footSpacerCylinderInternal(state);
    	spacerState = state;
	}
	public boolean getSpacerState() {
    	return spacerState;
	}
    public void setOffsets() {
    	this.armEncoderOffset = getArmEncoderCountInternal();
    	this.elevatorEncoderOffset = getElevatorEncoderCountInternal();
	}

	public void clearOffsets() {
    	this.armEncoderOffset = 0;
    	this.elevatorEncoderOffset = 0;
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

}

