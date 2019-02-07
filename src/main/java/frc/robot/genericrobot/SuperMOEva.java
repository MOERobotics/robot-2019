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

public class SuperMOEva extends GenericRobot {

    //Drive
    TalonSRX driveLA = new TalonSRX(0) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveLB = new TalonSRX(15) {{setNeutralMode(NeutralMode.Brake);}};
    //TalonSRX driveLC = new TalonSRX(15) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveRA = new TalonSRX(1) {{setNeutralMode(NeutralMode.Brake);}};
    TalonSRX driveRB = new TalonSRX(14) {{setNeutralMode(NeutralMode.Brake);}};
    //TalonSRX driveRC = new TalonSRX(15) {{setNeutralMode(NeutralMode.Brake);}};

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

    //Cargo/Hatch?

    //Hab Lifter
    CANSparkMax froggerLA = new CANSparkMax(20, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax froggerLB = new CANSparkMax(20, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax froggerRA = new CANSparkMax(20, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax froggerRB = new CANSparkMax(20, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANEncoder frogLAencoder = new CANEncoder(turret);
    CANEncoder frogLBencoder = new CANEncoder(turret);
    CANEncoder frogRAencoder = new CANEncoder(turret);
    CANEncoder frogRBencoder = new CANEncoder(turret);

    //Drive Functions
    public void setDrivePowerInternal(double leftMotor, double rightMotor) {
        driveLA.set(ControlMode.PercentOutput, leftMotor);
        driveLB.set(ControlMode.PercentOutput, leftMotor);
        //driveLC.set(ControlMode.PercentOutput, leftMotor);

        driveRA.set(ControlMode.PercentOutput, rightMotor);
        driveRB.set(ControlMode.PercentOutput, rightMotor);
        //driveRC.set(ControlMode.PercentOutput, leftMotor);
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
        //other things?
    }

    @Override
    public void resetDriveEncoder() {
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
    public void driveTurret(double power) {
        if(power > 1.0) power = 1.0;
        if(power < -1.0) power = -1.0;

        turret.set(power);
    }

    public void driveElevator(double power) {
        if(power > 1.0) power = 1.0;
        if(power < -1.0) power = -1.0;

		/*if(elevatorBottomLimitSwitch.get()) {
			if(power < 0) power = 0;
		}
		else if(elevatorTopLimitSwitch.get()) {
			if(power > 0) power = 0;
		}*/

        elevator.set(power);
    }

    public void driveArm(double power) {
        if(power > 1.0) power = 1.0;
        if(power < -1.0) power = -1.0;

        arm.set(power);
    }

    /*public void stopTurret() {
        turret.stopMotor();
        elevator.stopMotor();
        arm.stopMotor();
    }*/

    //Hab Climb
    public void climb() {

    }


}
