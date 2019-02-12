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
//import frc.robot.genericrobot.CaMOElot;
//import frc.robot.genericrobot.MOErio;
import frc.robot.genericrobot.SuperMOEva;

import edu.wpi.first.wpilibj.GenericHID.Hand;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;

import com.revrobotics.*;

public class Robot extends TimedRobot {

  GenericRobot robotHardware = new CaMOElot();
  Joystick driveStick = new Joystick(0);
  private XboxController functionStick = new XboxController(1);

  GenericAuto autoProgram = new DriveStraightAuto();

  //lidar
  SerialPort Blinky;
  boolean PortOpen = false;
  public static int numSensors = 8;
  public static int[] lidar = new int[numSensors];

  //drive elevator
  static final double upperElevator = 1;
  static final double bottomElevator = -0.6;

  //camera
  UsbCamera cam1;
  //UsbCamera cam2;
  VideoSink server;

  /* kP = 0.1, kI = 8*10^-3, kD = 0.0*/


  @Override
  public void robotInit() {
    autoProgram.robot = robotHardware;

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

    //CameraServer.getInstance().startAutomaticCapture();
    cam1 = CameraServer.getInstance().startAutomaticCapture(0);
    cam1.setWhiteBalanceAuto();
    /*cam2 = CameraServer.getInstance().startAutomaticCapture(1);
    cam2.setWhiteBalanceAuto();*/
    server = CameraServer.getInstance().getServer();
    CameraServer.getInstance().startAutomaticCapture();

    robotHardware.dashboardInit(robotHardware);
  }

    @Override
    public void robotPeriodic () {
      robotHardware.dashboardPeriodic(robotHardware);
    }

    @Override
    public void disabledInit () {
    }

    @Override
    public void disabledPeriodic () {
      if (driveStick.getRawButton(2)) {
        robotHardware.resetYaw();
        robotHardware.resetDriveEncoders();
      }

      robotHardware.dashboardPeriodic(robotHardware);
    }

    @Override
    public void autonomousInit () {
      autoProgram.init();
      //AutoTest.init();
      Lidar.init(Blinky);

      robotHardware.dashboardPeriodic(robotHardware);
    }

    @Override
    public void autonomousPeriodic () {
      robotHardware.checkSafety();
      autoProgram.run();
      //AutoTest.run(robotHardware);
      Lidar.getLidar(this, Blinky);
    }

    @Override
    public void teleopInit () {
      Lidar.init(Blinky);

      //robotHardware.dashboardInit(robotHardware);
    }

    @Override
    public void teleopPeriodic () {
      Lidar.getLidar(this, Blinky);

      //driving
      if (driveStick.getRawButton(2)) {
        robotHardware.turnRightInplace(.2);
      } else if (driveStick.getRawButton(3)) {
        robotHardware.moveBackward(.2); /*(0,4)*/
      } else if (driveStick.getRawButton(4)) {
        robotHardware.turnLeftInplace(.2);
      } else if (driveStick.getRawButton(5)) {

      }

      //just for now
      else if (driveStick.getRawButton(6)) {
        robotHardware.driveSA(0.5);
      } else if (driveStick.getRawButton(7)) {
        robotHardware.driveSB(0.5);
      } else if (driveStick.getRawButton(8)) {
        robotHardware.driveFA(0.5);
      } else if (driveStick.getRawButton(9)) {
        robotHardware.driveFB(0.5);
      }
      //fin~

      else if (driveStick.getTrigger()) {
        robotHardware.moveForward(.2); /*(0,4)*/
      } else {
        double driveJoyStickX = driveStick.getX();
        double driveJoyStickY = -driveStick.getY();

        double drivePowerLeft = driveJoyStickY + driveJoyStickX;
        double drivePowerRight = driveJoyStickY - driveJoyStickX;

        robotHardware.setDrivePower(drivePowerLeft, drivePowerRight);
      }

      //roller
      if(functionStick.getAButton()) robotHardware.rollIn();
      else if(functionStick.getBButton()) robotHardware.rollOut();
      else robotHardware.driveRoll(0);

      //hatch grab
      if(functionStick.getXButton()) robotHardware.grabHatch();
      else if(functionStick.getYButton()) robotHardware.grabHatch();//heh no

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
      }
      else {
        robotHardware.driveElevator((upperElevator * functionStick.getTriggerAxis(Hand.kRight)));
      }

      robotHardware.dashboardPeriodic(robotHardware);
    }

    @Override
    public void testInit () {
    }

    @Override
    public void testPeriodic () {
    }


}
