package frc.robot.genericrobot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SuperMOEva extends GenericRobot {
    final int COUNTS_PER_REV = 512;
    final double TICKS_TO_INCHES = 218;

    //Drive
    /*TalonSRX driveFreeA    = new TalonSRX(12) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveFreeB    = new TalonSRX(13) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveSupportA = new TalonSRX(14) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveSupportB = new TalonSRX(15) {{setNeutralMode(NeutralMode.Brake);}};*/

    TalonSRX driveLA    = new TalonSRX(12) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveLB    = new TalonSRX(13) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveRA = new TalonSRX(14) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveRB = new TalonSRX(15) {{setNeutralMode(NeutralMode.Brake);}};

    AHRS navX = new AHRS(SPI.Port.kMXP, (byte) 50);
    Encoder encoderL = new Encoder(0, 1, true, EncodingType.k4X);
    Encoder encoderR = new Encoder(2, 3, true, EncodingType.k4X); //falcon only
    //Encoder encoderR = new Encoder(4, 5, true, EncodingType.k4X); //supermoeva


    DoubleSolenoid shifter = new DoubleSolenoid(0, 1);

    //Turret
    CANSparkMax elevator = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax turret;//   = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax arm      = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANEncoder encoderElev = new CANEncoder(elevator);
    CANEncoder encoderTur;//  = new CANEncoder(turret);
    CANEncoder encoderArm  = new CANEncoder(arm);

    //Cargo/Hatch
    TalonSRX rollL = new TalonSRX(11) {{setNeutralMode(NeutralMode.Brake);}}; //aka the accumulators
    TalonSRX rollR = new TalonSRX(10) {{setNeutralMode(NeutralMode.Brake);}};

    Solenoid spearShaft = new Solenoid(2); //extend
    Solenoid spearHook  = new Solenoid(3); //grab
    Solenoid betaClimb  = new Solenoid(4); //grab
    Solenoid betaClimb2 = new Solenoid(5); //grab

    //Hab Lifter
    CANSparkMax froggerSA ;//= new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax froggerSB ;//= new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax froggerFA ;//= new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax froggerFB ;//= new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANEncoder encoderFrogL ;//= new CANEncoder(froggerSA);
    CANEncoder encoderFrogR ;//= new CANEncoder(froggerFA);

    {//not sure which side is inverted
        driveLA.setInverted(true);
        driveLB.setInverted(true);
        rollL.setInverted(true);
        arm.setIdleMode(CANSparkMax.IdleMode.kBrake);
        elevator.setIdleMode(CANSparkMax.IdleMode.kBrake);
        //turret.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    //lidar
    @Override
    public int numSensors() {
        return 8;
    }


    //Drive Functions
    public void setDrivePowerInternal(double leftMotor, double rightMotor) {
        driveRA.set(ControlMode.PercentOutput, leftMotor);
        driveRB.set(ControlMode.PercentOutput, leftMotor);

        driveLA.set(ControlMode.PercentOutput, rightMotor);
        driveLB.set(ControlMode.PercentOutput, rightMotor);
    }

    //shifting
    @Override
    public void shiftDriveInternal(DoubleSolenoid.Value value) {
        shifter.set(value);
    }

    //testing individual motors (if you need to do this, set the talons to COAST)
    /*public void driveRA(double power) {
        driveRA.set(ControlMode.PercentOutput, power);
    }

    public void driveRB(double power) {
        driveRB.set(ControlMode.PercentOutput, power);
    }

    public void driveLA(double power) {
        driveLA.set(ControlMode.PercentOutput, power);
    }

    public void driveLB(double power) {
        driveLB.set(ControlMode.PercentOutput, power);
    }*/

    @Override
    public double getDistanceLeftInches() {
        return encoderL.getRaw() / TICKS_TO_INCHES;
    }

    @Override
    public double getDistanceRightInches() {
        return encoderR.getRaw() / TICKS_TO_INCHES;
    }

    @Override
    public double getHeadingDegrees() {
        return navX.getYaw();
    }

    @Override
    public void stopEverything() {
        setDrivePowerInternal(0,0);
        setElevatorInternal(0);
        setTurretInternal(0);
        setArmInternal(0);
    }

    @Override
    public void resetDriveEncoders() {
        encoderL.reset();
        encoderR.reset();
    }

    @Override
    public void resetYaw() {
        navX.reset();
    }

    @Override
    public void stopDriving() {
        setDrivePowerInternal(0,0);
    }

    //Turret Functions
    @Override
    public void setElevatorInternal(double power) {
        elevator.set(power);
    }

    @Override
    public void enableElevatorLimits(boolean enabled) {
        elevator.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).enableLimitSwitch(enabled);
        elevator.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).enableLimitSwitch(enabled);
    }

    @Override
    public boolean isElevForwardLimitEnabled() {
        return elevator.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).isLimitSwitchEnabled();
    }

    @Override
    public boolean isElevReverseLimitEnabled() {
        return elevator.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).isLimitSwitchEnabled();
    }

    @Override
    public boolean atElevForwardLimit() {
        return elevator.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).get();
    }

    @Override
    public boolean atElevReverseLimit() {
        return elevator.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).get();
    }

    @Override
    public void setTurretInternal(double power) {
        //turret.set(power);
    }

    @Override
    public void setArmInternal(double power) {
        arm.set(power);
    }

    @Override
    public void enableArmLimits(boolean enabled) {
        arm.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).enableLimitSwitch(enabled);
        arm.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).enableLimitSwitch(enabled);
    }

    @Override
    public boolean isArmForwardLimitEnabled() {
        return arm.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).isLimitSwitchEnabled();
    }

    @Override
    public boolean isArmReverseLimitEnabled() {
        return arm.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).isLimitSwitchEnabled();
    }

    @Override
    public boolean atArmForwardLimit() {
        return arm.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).get();
    }

    @Override
    public boolean atArmReverseLimit() {
        return arm.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).get();
    }

   //Cargo/Hatch
    @Override
    public void setRollerInternal(double power) {
        rollL.set(ControlMode.PercentOutput, power);
        rollR.set(ControlMode.PercentOutput, power);
    }

    @Override
    public void shiftSpearShaftInternal(boolean out) {
        spearShaft.set(out);
    }

    @Override
    public void shiftSpearHookInternal(boolean out) {
        spearHook.set(out);
    }

    //Hab Climb
    /* rip frogger 2019-2019
    public void climbInternal(double power) {
        double deltaEncoder =
            encoderFrogL.getPosition() -
            encoderFrogR.getPosition();
        double
             leftPower = power,
            rightPower = power;
        if (power < 0 && deltaEncoder > 10) {
            leftPower = 0;
        }
        if (power > 0 && deltaEncoder < 10) {
            rightPower = 0;
        }
        froggerSA.set( leftPower);
        froggerSB.set( leftPower);
        froggerFA.set(rightPower);
        froggerFB.set(rightPower);
    }*/

    public void climbInternal(double power) {
        if (power > 0) {
            betaClimb.set(true);
        } else if (power < 0) {
            betaClimb.set(false);
        }
    }

    //Safety Check
    @Override
    public void checkSafety() {

        if (isElevatorUp  ()) driveElevator(0);
        if (isElevatorDown()) driveElevator(0);

        if (isTurretRight()) driveTurret(0);
        if (isTurretLeft ()) driveTurret(0);

        if (isArmUp  ()) driveArm(0);
        if (isArmDown()) driveArm(0);

    }

    @Override
    public boolean isElevatorUp() {
        return !getSafetyOverride() && getElevatorEncoderCount() >= 44;
    }

    @Override
    public boolean isElevatorDown() { return !getSafetyOverride() && getElevatorEncoderCount() <= -30; }

    @Override
    public boolean isTurretRight() {
        return false;
        //return !getSafetyOverride() && getTurretEncoderCount() >= 110;
    }

    @Override
    public boolean isTurretLeft() {
        return false;
        //return !getSafetyOverride() && getTurretEncoderCount() <= -5;
    }

    @Override
    public boolean isArmUp() {
        return !getSafetyOverride() && getArmEncoderCount() >= 100;
    }

    @Override
    public boolean isArmDown() {
        return !getSafetyOverride() && getArmEncoderCount() <= -2;
    }

    @Override
    public double getElevatorEncoderCountInternal() {
        return encoderElev.getPosition();
    }

    @Override
    public double getTurretEncoderCountInternal() {
        return 0;
        //return encoderTur.getPosition();
    }

    @Override
    public double getArmEncoderCountInternal() {
        return encoderArm.getPosition();
    }

    @Override
    public double getPitchDegrees() {
        return navX.getPitch();
    }

    @Override
    public double getRollDegrees() {
        return navX.getRoll();
    }

    public void climb2(boolean state) {
        betaClimb2.set(state);
    }

}
