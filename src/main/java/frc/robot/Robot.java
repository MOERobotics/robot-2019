package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.CounterBase.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {


  Joystick driveStick = new Joystick(0);


  TalonSRX driveLA = new TalonSRX(12) {{
    setNeutralMode(NeutralMode.Brake);
  }};
  TalonSRX driveLB = new TalonSRX(13) {{
    setNeutralMode(NeutralMode.Brake);
  }};
  TalonSRX driveLC = new TalonSRX(14) {{
    setNeutralMode(NeutralMode.Brake);
  }};
  TalonSRX driveRA = new TalonSRX(1) {{
    setNeutralMode(NeutralMode.Brake);
  }};
  TalonSRX driveRB = new TalonSRX(2) {{
    setNeutralMode(NeutralMode.Brake);
  }};
  TalonSRX driveRC = new TalonSRX(3) {{
    setNeutralMode(NeutralMode.Brake);
  }};

  AHRS navX = new AHRS(SPI.Port.kMXP, (byte) 50);
  Encoder distanceL = new Encoder(0, 1, false, EncodingType.k1X);
  Encoder distanceR = new Encoder(2, 3, true, EncodingType.k1X);

  double driveOutputLeft = 0.0, driveOutputRight = 0.0;


  @Override
  public void robotInit() {
    driveRA.setInverted(true);
    driveRB.setInverted(true);
    driveRC.setInverted(true);

  }

  @Override
  public void robotPeriodic() {
SmartDashboard.putNumber("LeftEncoder", distanceL.getDistance());
    SmartDashboard.putNumber("RightEncoder", distanceR.getDistance());
    SmartDashboard.putNumber("NavX  x distance value", navX.getDisplacementX());
  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
    if (driveStick.getRawButton(2)) {
      resetEncoders();
      navX.zeroYaw();
    }
  }

  @Override
  public void autonomousInit() {
    navX.resetDisplacement();
  }

  @Override

  public void autonomousPeriodic() {

    if (driveStick.getRawButton(4)) {
    }
        driveRobot(0.5, 0.5);
        if (distanceL.getDistance() >= 13440){

          driveRobot(0,0);
      }

     SmartDashboard.putNumber("NavX  x distance value", navX.getDisplacementX());

  }




  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {
    double yJoy = -driveStick.getY();
    double xJoy = driveStick.getX();

    if (driveStick.getTrigger()) {
      driveRobot(yJoy, yJoy);
    } else if (driveStick.getRawButton(2)) { // turn robot left
      driveRobot(-0.3, 0.3);
    } else if (driveStick.getRawButton(4)) {
      driveRobot(0.3, -0.3);
    } else {
      double left = yJoy + xJoy;
      double right = yJoy - xJoy;
      driveRobot(left, right);
    }

  }

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {
  }
  void driveRobot(double leftPower, double rightPower) {
    driveOutputLeft = leftPower;
    driveOutputRight = rightPower;
    driveLA.set(ControlMode.PercentOutput, leftPower);
    driveLB.set(ControlMode.PercentOutput, leftPower);
    driveLC.set(ControlMode.PercentOutput, leftPower);
    driveRA.set(ControlMode.PercentOutput, rightPower);
    driveRB.set(ControlMode.PercentOutput, rightPower);
    driveRC.set(ControlMode.PercentOutput, rightPower);
  }
  public void resetEncoders() {
    distanceL.reset();
    distanceR.reset();
  }
}