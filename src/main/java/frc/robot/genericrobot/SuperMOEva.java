package frc.robot.genericrobot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SuperMOEva extends GenericRobot {

    final int COUNTS_PER_REV = 512;
    final double TICKS_TO_INCHES = 178;

    //Drive
    TalonSRX driveLA = new TalonSRX(12) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveLB = new TalonSRX(13) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveRA = new TalonSRX(14) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveRB = new TalonSRX(15) {{setNeutralMode(NeutralMode.Brake);}};

    AHRS navX = new AHRS(SPI.Port.kMXP, (byte) 50);
    Encoder encoderL = new Encoder(0, 1, true, EncodingType.k4X);
    Encoder encoderR = new Encoder(4, 5, true, EncodingType.k4X);

    DoubleSolenoid shifter = new DoubleSolenoid(0, 1);

    //Turret
    CANSparkMax elevator = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax arm = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    //TalonSRX arm = new TalonSRX(4) {{setNeutralMode(NeutralMode.Brake);}};

    CANEncoder encoderElev = new CANEncoder(elevator);
    CANEncoder encoderArm = new CANEncoder(arm);
    //Encoder encoderArm = new Encoder(8, 9, true, EncodingType.k2X);

    //Cargo/Hatch
    TalonSRX rollL = new TalonSRX(11) {{setNeutralMode(NeutralMode.Brake);}}; //aka the accumulators
    TalonSRX rollR = new TalonSRX(10) {{setNeutralMode(NeutralMode.Brake);}};

    Solenoid spearShaft = new Solenoid(2); //extend
    Solenoid spearHook  = new Solenoid(3); //grab
    //Solenoid floorPickup = new Solenoid(4);
    DoubleSolenoid floorPickup = new DoubleSolenoid(6, 7); //GET NUMBERS
    Solenoid betaClimb  ;//= new Solenoid(4);
    Solenoid betaClimb2 = new Solenoid(5); //grab

    //Hab Lifter
    CANSparkMax froggerLA = new CANSparkMax(20, CANSparkMaxLowLevel.MotorType.kBrushless);//-Brian
    CANSparkMax froggerLB = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax froggerRA = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax froggerRB = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANEncoder encoderFrogL = new CANEncoder(froggerLA);
    CANEncoder encoderFrogR = new CANEncoder(froggerRA);

    {//not sure which side is inverted
        driveLA.setInverted(true);
        driveLB.setInverted(true);
        rollL.setInverted(true);
        arm.setIdleMode(CANSparkMax.IdleMode.kBrake);
        elevator.setIdleMode(CANSparkMax.IdleMode.kBrake);
        froggerLA.setIdleMode(CANSparkMax.IdleMode.kCoast);
        froggerLB.setIdleMode(CANSparkMax.IdleMode.kCoast);
        froggerRA.setIdleMode(CANSparkMax.IdleMode.kCoast);
        froggerRB.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    //lidar
    @Override
    public int numSensors() {
        return 2;
    }

    //Drive Functions
    public void setDrivePowerInternal(double leftMotor, double rightMotor) {
        driveRA.set(ControlMode.PercentOutput, leftMotor);
        driveRB.set(ControlMode.PercentOutput, leftMotor);

        driveLA.set(ControlMode.PercentOutput, rightMotor);
        driveLB.set(ControlMode.PercentOutput, rightMotor);
    }

    @Override
    public void stopEverything() {
        setDrivePowerInternal(0,0);
        setElevatorInternal(0);
        setArmInternal(0);
    }

    @Override
    public void stopDriving() {
        setDrivePowerInternal(0,0);
    }

    //shifting
    @Override
    public void shiftDriveInternal(DoubleSolenoid.Value value) {
        shifter.set(value);
    }

    //Encoder/NavX
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
    public double getPitchDegrees() {
        return navX.getPitch();
    }

    @Override
    public double getRollDegrees() {
        return navX.getRoll();
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

    /*@Override
    public boolean atElevForwardLimit() {
        return elevator.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).get();
    }

    @Override
    public boolean atElevReverseLimit() {
        return elevator.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).get();
    }*/

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

    /*@Override
    public boolean atArmForwardLimit() {
        return arm.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).get();
    }

    //@Override
    public boolean atArmReverseLimit() {
        return arm.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).get();
    }*/

    @Override
    public double getElevatorEncoderCountInternal() {
        return encoderElev.getPosition();
    }

    @Override
    public double getArmEncoderCountInternal() {
        return encoderArm.getPosition();
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

    @Override
    public void shiftFloorPickupInternal(DoubleSolenoid.Value out) {
        floorPickup.set(out);
    }

    //Hab Climb
    //rip frogger 2019-2019 //just kidding frogger is alive again! hallelujah
    public void climbInternal(double power) {
        //double deltaEncoder =
        //    encoderFrogL.getPosition() -
        //    encoderFrogR.getPosition();
        double
             leftPower = power,
            rightPower = power;
        //if (power < 0 && deltaEncoder > 10) {
        //    leftPower = 0;
        //}
        //if (power > 0 && deltaEncoder < 10) {
         //   rightPower = 0;
        //}
        if (navX.getRoll()<-3) {
            froggerLA.set(0.9*leftPower);
            froggerLB.set(0.9*leftPower);
            froggerRA.set(rightPower);
            froggerRB.set(rightPower);
        }
        else if (navX.getRoll()>3) {
            froggerLA.set(leftPower);
            froggerLB.set(leftPower);
            froggerRA.set(0.9*rightPower);
            froggerRB.set(0.9 *rightPower);
        } else {
            froggerLA.set(leftPower);
            froggerLB.set(leftPower);
            froggerRA.set(rightPower);
            froggerRB.set(rightPower);
        }

        SmartDashboard.putNumber("Left Power", leftPower);
        SmartDashboard.putNumber("Right Power", rightPower);

    }

    /*public void climbInternal(double power) {
        if (power > 0) {
            betaClimb.set(true);
        } else if (power < 0) {
            betaClimb.set(false);
        }
    }*/

    public void climbSupportUp(double power) {
        froggerLA.set(power);
        froggerLB.set(power);
        froggerRA.set(0);
        froggerRB.set(0);
    }

    public void climbFreeUp(double power) {
        froggerLA.set(0);
        froggerLB.set(0);
        froggerRA.set(power);
        froggerRB.set(power);
    }

    @Override
    public double getClimberLEncoderCount() {
        return encoderFrogL.getPosition();
    }

    @Override
    public double getClimberREncoderCount() {
        return encoderFrogR.getPosition();
    }

    //Safety Check
    @Override
    public void checkSafety() {
        if (isElevatorUp  ()) driveElevator(0);
        if (isElevatorDown()) driveElevator(0);

        if (isArmUp  ()) driveArm(0);
        if (isArmDown()) driveArm(0);

    }

    @Override
    public boolean isElevatorUp() {
        return false;
        //return elevator.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).get();
        //return (!getSafetyOverride() && getElevatorEncoderCount() >= 44) || atElevForwardLimit();
    }

    @Override
    public boolean isElevatorDown() {
        return false;
        //return elevator.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).get();
        //return (!getSafetyOverride() && getElevatorEncoderCount() <= -30) || atElevReverseLimit();
    }

    @Override
    public boolean isArmUp() {
        return false;
        //return arm.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).get();
        //return !getSafetyOverride() && getArmEncoderCount() >= 100 || atArmForwardLimit();
    }

    @Override
    public boolean isArmDown() {
        return false;
        //return arm.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).get();
        //return !getSafetyOverride() && getArmEncoderCount() <= -2 || atArmReverseLimit();
    }

    public void climb2(boolean state) {
        betaClimb2.set(state);
    }

}
