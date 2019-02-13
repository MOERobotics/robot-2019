package frc.robot.genericrobot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class SuperMOEva extends GenericRobot {

    final int COUNTS_PER_REV = 512;

    //Drive
    TalonSRX driveFreeA = new TalonSRX(12) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveFreeB = new TalonSRX(13) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveSupportA = new TalonSRX(14) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveSupportB = new TalonSRX(15) {{setNeutralMode(NeutralMode.Brake);}};

    AHRS navX = new AHRS(SPI.Port.kMXP, (byte) 50);
    Encoder encoderL = new Encoder(0, 1, true, EncodingType.k1X);
    Encoder encoderR = new Encoder(2, 3, true, EncodingType.k1X);

    DoubleSolenoid shifter = new DoubleSolenoid(0, 1);

    {//not sure which side is inverted
        driveFreeA.setInverted(true);
        driveFreeB.setInverted(true);
    }

    //Turret
    CANSparkMax elevator = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax turret = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax arm = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANEncoder encoderElev = new CANEncoder(elevator);
    CANEncoder encoderTur = new CANEncoder(turret);
    CANEncoder encoderArm = new CANEncoder(arm);

    //DigitalInput elevatorBottomLimitSwitch = new DigitalInput(6);
    //DigitalInput elevatorTopLimitSwitch = new DigitalInput(7);

    //Cargo/Hatch
    TalonSRX rollL;// = new TalonSRX(0) {{setNeutralMode(NeutralMode.Brake);}}; //aka the accumulators
    TalonSRX rollR;// = new TalonSRX(0) {{setNeutralMode(NeutralMode.Brake);}};

    DoubleSolenoid hatchGrabberA = new DoubleSolenoid(0, 1);
    DoubleSolenoid hatchGrabberB = new DoubleSolenoid(2, 3);

    //Hab Lifter
    CANSparkMax froggerLA;// = new CANSparkMax(30, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax froggerLB;// = new CANSparkMax(31, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax froggerRA;// = new CANSparkMax(32, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax froggerRB;// = new CANSparkMax(33, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANEncoder encoderFrogLA = new CANEncoder(froggerLA);
    CANEncoder encoderFrogLB = new CANEncoder(froggerLB);
    CANEncoder encoderFrogRA = new CANEncoder(froggerRA);
    CANEncoder encoderFrogRB = new CANEncoder(froggerRB);


    //Drive Functions
    public void setDrivePowerInternal(double leftMotor, double rightMotor) {
        driveSupportA.set(ControlMode.PercentOutput, leftMotor);
        driveSupportB.set(ControlMode.PercentOutput, leftMotor);

        driveFreeA.set(ControlMode.PercentOutput, rightMotor);
        driveFreeB.set(ControlMode.PercentOutput, rightMotor);
    }

    //testing individual motors
    public void driveSA(double power) {
        driveSupportA.set(ControlMode.PercentOutput, power);
    }

    public void driveSB(double power) {
        driveSupportB.set(ControlMode.PercentOutput, power);
    }

    public void driveFA(double power) {
        driveFreeA.set(ControlMode.PercentOutput, power);
    }

    public void driveFB(double power) {
        driveFreeB.set(ControlMode.PercentOutput, power);
    }

    //fin

    @Override
    public double getDistanceLeftInches() {
        return encoderL.getRaw();
    }

    @Override
    public double getDistanceRightInches() {
        return encoderR.getRaw();
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
    public void setTurretInternal(double power) {
        turret.set(power);
    }

    @Override
    public void setArmInternal(double power) {
        arm.set(power);
    }



    /*public void stopTurret() {
        turret.stopMotor();
        elevator.stopMotor();
        arm.stopMotor();
    }*/

   //Cargo/Hatch
    @Override
    public void setRollerInternal(double power) {
        //rollL.set(ControlMode.PercentOutput, power);
        //rollR.set(ControlMode.PercentOutput, power);
    }

    //Hab Climb

    public void driveFroggers(double power) {
        //froggerLA.set(power);
        //froggerLB.set(power);
        //froggerRA.set(power);
        //froggerRB.set(power);
    }

    public void climb() {
        if (encoderFrogLA.getPosition() == encoderFrogLB.getPosition()  && encoderFrogLA.getPosition()  == encoderFrogRA.getPosition()
        && encoderFrogLA.getPosition()  == encoderFrogRB.getPosition()) {
            if (encoderFrogLA.getPosition() < 100) driveFroggers(1);
        }
    }

    //Safety Check
    @Override
    public void checkSafety() {
        /*if(elevatorBottomLimitSwitch.get()) {
			if(power < 0) power = 0;
		}
		else if(elevatorTopLimitSwitch.get()) {
			if(power > 0) power = 0;
		}*/

        if (isElevatorUp()) driveElevator(0);
        if (isElevatorDown()) driveElevator(0);

        if (isTurretRight()) driveTurret(0);
        if (isTurretLeft()) driveTurret(0);

        if (isArmUp()) driveArm(0);
        if (isArmDown()) driveArm(0);

        if (!froggersAreInSync()) driveFroggers(0);
    }

    @Override
    public boolean isElevatorUp() {
        return encoderElev.getPosition() <= -33.6;
    }

    @Override
    public boolean isElevatorDown() {
        return encoderElev.getPosition() >= 33.6;
    }

    @Override
    public boolean isTurretRight() {
        return encoderTur.getPosition() >= 180;
    }

    @Override
    public boolean isTurretLeft() {
        return encoderTur.getPosition() <= 0;
    }

    @Override
    public boolean isArmUp() {
        return encoderArm.getPosition() <= 0;
    }

    @Override
    public boolean isArmDown() {
        return encoderArm.getPosition() >= 67.2;
    }


    public boolean froggersAreInSync() {
        return encoderFrogLA.getPosition() == encoderFrogLB.getPosition()  && encoderFrogLA.getPosition()  == encoderFrogRA.getPosition()
            && encoderFrogLA.getPosition()  == encoderFrogRB.getPosition();
    }

    @Override
    public double getElevatorEncoderCount() {
        return encoderElev.getPosition();
    }

    @Override
    public double getTurretEncoderCount() {
        return encoderTur.getPosition();
    }

    @Override
    public double getArmEncoderCount() {
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

    @Override
    public void shiftHigh() {
        shifter.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void shiftLow() {
        shifter.set(DoubleSolenoid.Value.kForward);
    }

}
