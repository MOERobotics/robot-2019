/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.CaMOElot;
import frc.robot.genericrobot.GenericRobot;
import frc.robot.genericrobot.MOErio;


public class Robot extends TimedRobot {

  GenericRobot robotHardware = new MOErio();
  Joystick leftJoystick = new Joystick(0);

  GenericAuto autoProgram = new MOErioCargoFrontGoBackAuto();

  /* kP = 0.1, kI = 8*10^-3, kD = 0.0*/


  @Override
  public void robotInit() {
    autoProgram.robot = robotHardware;
  }

  @Override
  public void robotPeriodic() {
      SmartDashboard.putNumber("Yaw: ",robotHardware.getHeadingDegrees());
      SmartDashboard.putNumber("Left Encoder (inches): ",robotHardware.getDistanceLeftInches());
      SmartDashboard.putNumber("Right Encoder (inches): ",robotHardware.getDistanceRightInches());
      SmartDashboard.putNumber("Left Drive Power: ",robotHardware.getLeftDrivePower());
      SmartDashboard.putNumber("Right Drive Power: ",robotHardware.getRightDrivePower());
      SmartDashboard.putNumber("autostep: ",autoProgram.autoStep);
      SmartDashboard.putNumber("Pitch: ", robotHardware.getPitch());
      SmartDashboard.putNumber("Roll: ", robotHardware.getRoll());

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    if(leftJoystick.getRawButton(2)){
      robotHardware.resetYaw();
      robotHardware.resetDriveEncoder();
    }
  }

  @Override
  public void autonomousInit() {
    autoProgram.init();
    //AutoTest.init();
  }

  @Override
  public void autonomousPeriodic() {
    autoProgram.run();
    //AutoTest.run(robotHardware);
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

    if (leftJoystick.getRawButton(2)) {
      robotHardware.moveForward(.2); /*(0,4)*/
    } else if (leftJoystick.getRawButton(3)) {
      robotHardware.moveBackward(.2); /*(0,4)*/
    } else if(leftJoystick.getRawButton(4)) {
      robotHardware.turnLeftInplace(.2);
    } else if(leftJoystick.getRawButton(5)) {
      robotHardware.turnRightInplace(.2);
    } else if(leftJoystick.getTrigger()){
      double driveJoyStickY = -leftJoystick.getY();
      robotHardware.setDrivePower(driveJoyStickY,driveJoyStickY);
    } else if(leftJoystick.getRawButton(1)){
      robotHardware.resetDriveEncoder();
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
