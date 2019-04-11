package frc.robot.genericrobot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.stream.Collectors;
import java.util.stream.Stream;

public class SuperMOEva extends GenericRobot {

    final int COUNTS_PER_REV = 512;
    final double TICKS_TO_INCHES = 462;
    final double ELEV_TICKS_TO_INCHES = 2.85;
    double elevatorEncoderStart = 0;
    double armEncoderStart;

    //Drive
    /*TalonSRX driveLA = new TalonSRX(12) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveLB = new TalonSRX(13) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveRA = new TalonSRX(14) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveRB = new TalonSRX(15) {{setNeutralMode(NeutralMode.Brake);}};

    AHRS navX = new AHRS(SPI.Port.kMXP, (byte) 50);
    Encoder encoderL = new Encoder(0, 1, true, EncodingType.k4X);
    Encoder encoderR = new Encoder(4, 5, true, EncodingType.k4X);*/

    //Turret
    //CANSparkMax elevator = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
    //CANSparkMax arm = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);

    //CANEncoder encoderElev = new CANEncoder(elevator);
    //CANEncoder encoderArm = new CANEncoder(arm);

    //Cargo/Hatch
    //TalonSRX rollL = new TalonSRX(11) {{setNeutralMode(NeutralMode.Brake);}};
    //TalonSRX rollR = new TalonSRX(10) {{setNeutralMode(NeutralMode.Brake);}};

    Solenoid footSpacerCylinder = new Solenoid(0);
    Solenoid spearShaft = new Solenoid(2); //extend
    Solenoid spearHook  = new Solenoid(3); //grab
    //Solenoid floorPickup = new Solenoid(4);
    DoubleSolenoid linearSlider = new DoubleSolenoid(5, 6);
    Solenoid shifter = new Solenoid(7);

    //Hab Lifter
    //CANSparkMax froggerLA = new CANSparkMax(20, CANSparkMaxLowLevel.MotorType.kBrushless);//-Brian
    //CANSparkMax froggerLB = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    //CANSparkMax froggerRA = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    //CANSparkMax froggerRB = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);

    //CANEncoder encoderFrogL = new CANEncoder(froggerLA);
    //CANEncoder encoderFrogR = new CANEncoder(froggerRA);

    DigitalInput elevatorTopLimit = new DigitalInput(6);
    DigitalInput elevatorBottomLimit = new DigitalInput(7);

    //DigitalInput climbLLimit = new DigitalInput(6);
    //DigitalInput climbRLimit = new DigitalInput(7);

    /*{
        driveLA.setInverted(true);
        driveLB.setInverted(true);
        rollL.setInverted(true);
        arm.setIdleMode(CANSparkMax.IdleMode.kBrake);
        elevator.setIdleMode(CANSparkMax.IdleMode.kBrake);
        froggerLA.setIdleMode(CANSparkMax.IdleMode.kCoast);
        froggerLB.setIdleMode(CANSparkMax.IdleMode.kCoast);
        froggerRA.setIdleMode(CANSparkMax.IdleMode.kCoast);
        froggerRB.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }*/


    //lidar
    @Override
    public int numSensors() {
        return 1;
    }
    //right 0, middle 1, left 2? lol check again

    //Drive Functions
    public void setDrivePowerInternal(double leftMotor, double rightMotor) {
        //driveRA.set(ControlMode.PercentOutput, leftMotor);
        //driveRB.set(ControlMode.PercentOutput, leftMotor);

        //driveLA.set(ControlMode.PercentOutput, rightMotor);
        //driveLB.set(ControlMode.PercentOutput, rightMotor);
    }

    @Override
    public void stopEverything() {
        //froggerLA.disable();
        //froggerLB.disable();
        //froggerRA.disable();
        //froggerRB.disable();
    }

    //shifting
    @Override
    public void shiftDriveInternal(boolean state) {
        shifter.set(state);
    }

    //Encoder/NavX
    @Override
    public double getDistanceLeftInches() {
        return 0;//-encoderL.getRaw() / TICKS_TO_INCHES;
    }

    @Override
    public double getDistanceRightInches() {
        return 0;//encoderR.getRaw() / TICKS_TO_INCHES;
    }

    @Override
    public double getHeadingDegrees() {
        return 0;//navX.getYaw();
    }

    @Override
    public double getPitchDegrees() {
        return 0;//navX.getPitch();
    }

    @Override
    public double getRollDegrees() {
        return 0;//navX.getRoll();
    }

    @Override
    public void resetDriveEncoders() {
        //encoderL.reset();
        //encoderR.reset();
    }

    @Override
    public void resetYaw() {
        //navX.reset();
    }

    @Override
    public void resetElevatorPosition() {
        //encoderElev.setPosition(0);
    }

    @Override
    public void resetArmPosition() {
        //encoderArm.setPosition(0);
    }

    @Override
    public void resetClimberPosition() {
        //encoderFrogL.setPosition(0);
        //encoderFrogR.setPosition(0);
    }

    @Override
    public void stopDriving() {
        setDrivePowerInternal(0,0);
    }

    //Turret Functions
    @Override
    public void setElevatorInternal(double power) {
        //elevator.set(power);
    }

    @Override
    public void enableElevatorLimits(boolean enabled) {
        //elevator.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).enableLimitSwitch(false);
        //elevator.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).enableLimitSwitch(false);
    }

    @Override
    public boolean isElevForwardLimitEnabled() {
        return false;
        //elevator.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).isLimitSwitchEnabled();
    }

    @Override
    public boolean isElevReverseLimitEnabled() {
        return false;
        //elevator.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).isLimitSwitchEnabled();
    }

    public boolean atElevForwardLimit() {
        return false;
        //return elevator.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).get();
    }

    public boolean atElevReverseLimit() {
        return false;
        //return elevator.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).get();
    }

    @Override
    public boolean atElevatorTopLimit() {
        return elevatorTopLimit.get();
    }

    @Override
    public boolean atElevatorBottomLimit() {
        return elevatorBottomLimit.get();
    }

    @Override
    public void setArmInternal(double power) {
        //arm.set(power);
    }

    @Override
    public void enableArmLimits(boolean enabled) {
        //arm.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).enableLimitSwitch(false);
        //arm.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).enableLimitSwitch(false);
        //if (enabled == false) Thread.dumpStack();
    }

    @Override
    public boolean isArmForwardLimitEnabled() {
        return false;// arm.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).isLimitSwitchEnabled();
    }

    @Override
    public boolean isArmReverseLimitEnabled() {
        return false;//arm.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).isLimitSwitchEnabled();
    }

    public boolean atArmForwardLimit() {
        return false;
        //return arm.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).get();
    }

    public boolean atArmReverseLimit() {
        return false;
        //return arm.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed).get();
    }

    @Override
    public double getElevatorEncoderCountInternal() {
        return 0;//encoderElev.getPosition();
    }

    @Override
    public double getArmEncoderCountInternal() {
        return 0; //encoderArm.getPosition();
    }
/*
    @Override
    public double getElevatorEncoderStart() {
        return elevatorEncoderStart;
    }

    @Override
    public double getArmEncoderStart() {
        return armEncoderStart;
    }
*/
    //Cargo/Hatch
    @Override
    public void setRollerInternal(double power) {
        //rollL.set(ControlMode.PercentOutput, power);
        //rollR.set(ControlMode.PercentOutput, power);
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
    public void shiftFloorPickupInternal(boolean out) {
        //floorPickup.set(out);
    }

    //Hab Climb

    public void climbInternal(double power) {
        //double deltaEncoder =
        //    encoderFrogL.getPosition() -
        //    encoderFrogR.getPosition();

        double ReductionFactor = 0.6; // MF 0.5; SM 0.9;
        double
             leftPower = power,
                rightPower = power;

        double angleTol = 0.75;
        double angleOrigin = 0;
        //if (climbLLimit.get()) leftPower = 0;
        //if (climbRLimit.get()) rightPower = 0;

        /*if (power<0) {
            if (navX.getRoll()-angleOrigin < -angleTol) {
                leftPower = ReductionFactor*leftPower;
            } else if (navX.getRoll() - angleOrigin > angleTol) {
                rightPower = ReductionFactor*rightPower;
            }
        }
        if (power>0) {
            if (navX.getRoll() - angleOrigin < -angleTol) {
                rightPower = ReductionFactor * rightPower;
            } else if (navX.getRoll() - angleOrigin > angleTol) {
                leftPower = ReductionFactor * leftPower;
            }
        }*/
        climbInternalK(leftPower, rightPower);
        SmartDashboard.putNumber("Left Climb Power", leftPower);
        SmartDashboard.putNumber("Right Climb Power", rightPower);
    }

    @Override
    public void climbLDown(double power) {
        //froggerLB.set(power);
        //froggerLA.set(power);
    }

    @Override
    public void climbRDown(double power) {
        //froggerRA.set(power);
        //froggerRB.set(power);
    }

    @Override
    public void getClimberCurrent() {
        //SmartDashboard.putNumber("climber la I: ", froggerLA.getOutputCurrent());
        //SmartDashboard.putNumber("climber lb I: ", froggerLB.getOutputCurrent());
        //SmartDashboard.putNumber("climber ra I: ", froggerRA.getOutputCurrent());
        //SmartDashboard.putNumber("climber rb I: ", froggerRB.getOutputCurrent());

    }
    double oldLeftK, oldRightK;
    public void climbInternalK(double leftPower, double rightPower) {
        if (oldLeftK != leftPower || oldRightK != rightPower) {
            /*for (StackTraceElement e : Thread.currentThread().getStackTrace()) {
                System.out.println(e);
            }
            System.out.printf("L:%f R:%f\n", leftPower, rightPower);*/
            oldRightK = rightPower;
            oldLeftK = leftPower;
        }
        //froggerLA.set(leftPower);
        //froggerLB.set(leftPower);
        //froggerRA.set(rightPower);
        //froggerRB.set(rightPower);
    }

    @Override
    public double getClimberLEncoderCount() {
        return 0;//encoderFrogL.getPosition();
    }

    @Override
    public double getClimberREncoderCount() {
        return 0;//encoderFrogR.getPosition();
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
        //return atElevForwardLimit();
        //return (!getSafetyOverride() && (getElevatorEncoderCount() >= 44) || atElevForwardLimit());
    }

    @Override
    public boolean isElevatorDown() {
        return false;
        //return atElevReverseLimit();
        //return !getSafetyOverride() && (getElevatorEncoderCount() <= -30 || atElevReverseLimit());
    }

    @Override
    public boolean isArmUp() {
        return false;
        //return atArmForwardLimit();
        //return !getSafetyOverride() && (getArmEncoderCount() >= 100 || atArmForwardLimit());
    }

    @Override
    public boolean isArmDown() {
        return false;
        //return atArmReverseLimit();
        //return !getSafetyOverride() && (getArmEncoderCount() <= -2 || atArmReverseLimit());
    }

    @Override
    public void linearSlideInternal(DoubleSolenoid.Value state) {
        linearSlider.set(state);
    }

    @Override
    public void footSpacerCylinderInternal(boolean state) {
        footSpacerCylinder.set(state);
    }
}
