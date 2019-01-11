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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {

  GenericRobot robotHardware = new CaMOElot();
  Joystick leftJoystick = new Joystick(0);


  @Override
  public void robotInit() {

  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
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
    SmartDashboard.putNumber("Yaw: ",robotHardware.getDegrees());
    SmartDashboard.putNumber("Left Encoder: ",robotHardware.getDistance());

    if (leftJoystick.getRawButton(2)) {
      robotHardware.moveForward(.4); /*(0,4)*/
    } else if (leftJoystick.getRawButton(3)) {
      robotHardware.moveBackward(.4); /*(0,4)*/
    } else if(leftJoystick.getRawButton(4)) {
      robotHardware.turnLeft(.4);
    } else if(leftJoystick.getRawButton(5)) {
      robotHardware.turnRight(.4);
    } else {
      double driveJoyStickX = leftJoystick.getX();
      double driveJoyStickY = -leftJoystick.getY();

      double drivePowerLeft = driveJoyStickY + driveJoyStickX;
      double drivePowerRight = driveJoyStickY - driveJoyStickX;

      robotHardware.setDrivePower(drivePowerLeft, drivePowerRight);
    }
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }



}
