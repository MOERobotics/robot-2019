/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.genericrobot.SuperMOEva;

import java.util.function.BooleanSupplier;
import java.util.function.Predicate;

public class Robot extends TimedRobot {

  GenericRobot robotHardware = new SuperMOEva();
  Joystick leftJoystick = new Joystick(0);
  private XboxController functionStick = new XboxController(1);

  GenericAuto autoProgram = new DriveStraightAuto();

  //lidar
  //SerialPort Blinky;
 // boolean PortOpen = false;
 // public static int numSensors = 8;
 // public static int[] lidar = new int[numSensors];

  //drive elevator
  static final double upperElevator = 1;
  static final double bottomElevator = -0.6;

  /* kP = 0.1, kI = 8*10^-3, kD = 0.0*/


  @Override
  public void robotInit() {
    autoProgram.robot = robotHardware;
/*
    //opening serial port
    if (!PortOpen) {
      PortOpen = true;

      try {
        Blinky = new SerialPort(9600, SerialPort.Port.kMXP, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
        SmartDashboard.putString("Open serial port: ", "Success!");
      } catch (Exception e) {
        String exception = e + "";
        SmartDashboard.putString("I caught: ", exception);
        PortOpen = false;
      }

    }
    */
  }

    @Override
    public void robotPeriodic () {
      SmartDashboard.putNumber("Yaw: "              , robotHardware.getHeadingDegrees());
      SmartDashboard.putNumber("Left Encoder: "     , robotHardware.getDistanceLeftInches());
      SmartDashboard.putNumber("Right Encoder: "    , robotHardware.getDistanceRightInches());
      SmartDashboard.putNumber("Left Drive Power: " , robotHardware.getLeftDrivePower());
      SmartDashboard.putNumber("Right Drive Power: ", robotHardware.getRightDrivePower());
      SmartDashboard.putNumber("Turret Encoder: "   , robotHardware.getTurretPower());
      SmartDashboard.putNumber("Elevator Encoder: " , robotHardware.getElevatorPower());
      SmartDashboard.putNumber("Arm Power: "        , robotHardware.getArmPower());
      SmartDashboard.putNumber("Roller Power: "     , robotHardware.getRollerPower());
      SmartDashboard.putNumber("autostep: "         , autoProgram.autoStep);
    }

    @Override
    public void disabledInit () {
    }

    @Override
    public void disabledPeriodic () {
      if (leftJoystick.getRawButton(2)) {
        robotHardware.resetYaw();
        robotHardware.resetDriveEncoders();
      }
    }

    @Override
    public void autonomousInit () {
      autoProgram.init();
      //AutoTest.init();
    }

    @Override
    public void autonomousPeriodic () {
      robotHardware.checkSafety();
      autoProgram.run();
      //AutoTest.run(robotHardware);
    }

    @Override
    public void teleopInit () {
    }

    @Override
    public void teleopPeriodic () {
      //driving
      if (leftJoystick.getRawButton(2) || leftJoystick.getTrigger()) {
        robotHardware.moveForward(.2); /*(0,4)*/
      } else if (leftJoystick.getRawButton(3)) {
        robotHardware.moveBackward(.2); /*(0,4)*/
      } else if (leftJoystick.getRawButton(4)) {
        robotHardware.turnLeftInplace(.2);
      } else if (leftJoystick.getRawButton(5)) {
        robotHardware.turnRightInplace(.2);
      }
      //just for now
      else if (leftJoystick.getRawButton(6)) {
        robotHardware.driveSA(0.5);
      } else if (leftJoystick.getRawButton(7)) {
        robotHardware.driveSB(0.5);
      } else if (leftJoystick.getRawButton(8)) {
        robotHardware.driveFA(0.5);
      } else if (leftJoystick.getRawButton(9)) {
        robotHardware.driveFB(0.5);
      } else {
        double driveJoyStickX =  leftJoystick.getX();
        double driveJoyStickY = -leftJoystick.getY();

        if (Math.abs(driveJoyStickX) < 0.15) driveJoyStickX = 0.0;

        double drivePowerLeft  = driveJoyStickY + driveJoyStickX;
        double drivePowerRight = driveJoyStickY - driveJoyStickX;

        robotHardware.setDrivePower(drivePowerLeft, drivePowerRight);
      }

      //roller
      if     (functionStick.getAButton()) robotHardware.rollIn (0.3);
      else if(functionStick.getBButton()) robotHardware.rollOut(0.3);
      else robotHardware.driveRoller(0);


      //arm
      if(functionStick.getBumper(Hand.kLeft)) robotHardware.driveArm(0.8);
      else if(functionStick.getBumper(Hand.kRight)) robotHardware.driveArm(-0.4);

      //turret
      if(functionStick.getStickButton(Hand.kRight)) robotHardware.driveTurret(0.5);
      else if (functionStick.getStickButton(Hand.kLeft)) robotHardware.driveTurret(-0.5);
      else robotHardware.driveTurret(0);

      //elevator
      if(functionStick.getTriggerAxis(Hand.kLeft) > functionStick.getTriggerAxis(Hand.kRight)) {
        robotHardware.driveElevator((bottomElevator * functionStick.getTriggerAxis(Hand.kLeft)));
      } else {
        robotHardware.driveElevator((upperElevator * functionStick.getTriggerAxis(Hand.kRight)));
      }
    }

    @Override
    public void testInit () {
    }

    @Override
    public void testPeriodic () {
    }

}
