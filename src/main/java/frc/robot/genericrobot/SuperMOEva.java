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

    //Drive
    TalonSRX driveLA = new TalonSRX(12) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveLB = new TalonSRX(13) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveRA = new TalonSRX(14) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveRB = new TalonSRX(15) {{setNeutralMode(NeutralMode.Brake);}};

    AHRS navX = new AHRS(SPI.Port.kMXP, (byte) 50);
    Encoder encoderL = new Encoder(0, 1, true, EncodingType.k1X);
    Encoder encoderR = new Encoder(2, 3, true, EncodingType.k1X);

    {
        driveRA.setInverted(true);
        driveRB.setInverted(true);
    }

    //Turret
    CANSparkMax turret = new CANSparkMax(19, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax arm = new CANSparkMax(21, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax elevator = new CANSparkMax(20, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANEncoder encoderTur = new CANEncoder(turret);
    CANEncoder encoderArm = new CANEncoder(arm);
    CANEncoder encoderElev = new CANEncoder(elevator);

    //DigitalInput elevatorBottomLimitSwitch = new DigitalInput(6);
    //DigitalInput elevatorTopLimitSwitch = new DigitalInput(7);

    //Cargo/Hatch
    TalonSRX rollL = new TalonSRX(12) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX rollR = new TalonSRX(3) {{setNeutralMode(NeutralMode.Brake);}};
    DoubleSolenoid hatchGrabberA = new DoubleSolenoid(0, 1);
    DoubleSolenoid hatchGrabberB = new DoubleSolenoid(2, 3);

    //Hab Lifter
    CANSparkMax froggerLA = new CANSparkMax(30, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax froggerLB = new CANSparkMax(31, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax froggerRA = new CANSparkMax(32, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax froggerRB = new CANSparkMax(33, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANEncoder encoderFrogLA = new CANEncoder(froggerLA);
    CANEncoder encoderFrogLB = new CANEncoder(froggerLB);
    CANEncoder encoderFrogRA = new CANEncoder(froggerRA);
    CANEncoder encoderFrogRB = new CANEncoder(froggerRB);

    //Solenoid flappyBoi = new Solenoid(3);


    //Drive Functions
    public void setDrivePowerInternal(double leftMotor, double rightMotor) {
        driveLA.set(ControlMode.PercentOutput, leftMotor);
        driveLB.set(ControlMode.PercentOutput, leftMotor);

        driveRA.set(ControlMode.PercentOutput, rightMotor);
        driveRB.set(ControlMode.PercentOutput, rightMotor);
    }

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

    @Override
    public void elevatorUp(double power) {
        setElevatorInternal(power);
    }

    @Override
    public void elevatorDown(double power) {
        setElevatorInternal(-power);
    }

    @Override
    public void turretRight(double power) {
        setTurretInternal(power);
    }

    @Override
    public void turretLeft(double power) {
        setTurretInternal(-power);
    }


    /*public void stopTurret() {
        turret.stopMotor();
        elevator.stopMotor();
        arm.stopMotor();
    }*/

    //Hab Climb

    public void driveFroggers(double power) {
        froggerLA.set(power);
        froggerLB.set(power);
        froggerRA.set(power);
        froggerRB.set(power);
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

        if (froggersAreInSync()) driveFroggers(0);
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


}
