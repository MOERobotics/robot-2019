/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.kauailabs.navx.frc.*;



public class Robot extends TimedRobot {
  TalonSRX testTalon;
  AHRS navx;

  TalonSRX driveLA   = new TalonSRX( 0) {{ setNeutralMode(NeutralMode.Brake); }};
  TalonSRX driveLB   = new TalonSRX(15) {{ setNeutralMode(NeutralMode.Brake); }};
  TalonSRX driveRA   = new TalonSRX( 1) {{ setNeutralMode(NeutralMode.Brake); }};
  TalonSRX driveRB   = new TalonSRX(14) {{ setNeutralMode(NeutralMode.Brake); }};


    AHRS         navX       = new AHRS(SPI.Port.kMXP, (byte) 50);
    Encoder      encoderL  = new Encoder(0, 1, true, EncodingType.k1X);
    Encoder      encoderR  = new Encoder(2, 3, true, EncodingType.k1X);

    private Joystick driveStick = new Joystick(0);

  @Override
  public void robotInit() {
      driveRA.setInverted(true);
      driveRB.setInverted(true);
  }

  @Override
  public void robotPeriodic() {
      statusMessage = "Everything is good!";
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
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
      double yJoy = -driveStick.getY();
      double xJoy = driveStick.getX();


      void driveRobot(double leftPower, double rightPower) {
          driveOutputLeft = leftPower;
          driveOutputRight = rightPower;
          driveLA.set(ControlMode.PercentOutput, leftPower);
          driveLB.set(ControlMode.PercentOutput, leftPower);
          driveRA.set(ControlMode.PercentOutput, rightPower);
          driveRB.set(ControlMode.PercentOutput, rightPower);
      }
      public void resetEncoders() {
          encoderL.reset();
          encoderR.reset();
      }
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }



}
