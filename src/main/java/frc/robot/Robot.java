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
import frc.robot.genericrobot.SuperMOEva;

import com.revrobotics.*;

public class Robot extends TimedRobot {

  GenericRobot robotHardware = new SuperMOEva();
  Joystick leftJoystick = new Joystick(0);

  GenericAuto autoProgram = new DriveStraightAuto();

  public static int numSensors = 2;
  public static int[] lidar = new int[numSensors];

  //lidar
  SerialPort Blinky;
  boolean PortOpen = false;

  //neo
  CANSparkMax azimuth = new CANSparkMax(19, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax elevation = new CANSparkMax(20, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax elevator = new CANSparkMax(21, CANSparkMaxLowLevel.MotorType.kBrushless);

  CANEncoder azimuthEncoder = new CANEncoder(azimuth);
  CANEncoder elevationEncoder = new CANEncoder(elevation);
  CANEncoder elevatorEncoder = new CANEncoder(elevator);


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
  }

    @Override
    public void robotPeriodic () {
      SmartDashboard.putNumber("Yaw: ", robotHardware.getHeadingDegrees());
      SmartDashboard.putNumber("Left Encoder: ", robotHardware.getDistanceLeftInches());
      SmartDashboard.putNumber("Right Encoder: ", robotHardware.getDistanceRightInches());
      SmartDashboard.putNumber("Left Drive Power: ", robotHardware.getLeftDrivePower());
      SmartDashboard.putNumber("Right Drive Power: ", robotHardware.getRightDrivePower());
      SmartDashboard.putNumber("autostep: ", autoProgram.autoStep);

    }

    @Override
    public void disabledInit () {
    }

    @Override
    public void disabledPeriodic () {
      if (leftJoystick.getRawButton(2)) {
        robotHardware.resetYaw();
      }
    }

    @Override
    public void autonomousInit () {
      autoProgram.init();
      //AutoTest.init();
      Lidar.init(Blinky);
    }

    @Override
    public void autonomousPeriodic () {
      autoProgram.run();
      //AutoTest.run(robotHardware);
      Lidar.getLidar(this, Blinky);
    }

    @Override
    public void teleopInit () {
      Lidar.init(Blinky);
    }

    @Override
    public void teleopPeriodic () {
      Lidar.getLidar(this, Blinky);

      if (leftJoystick.getRawButton(2)) {
        robotHardware.moveForward(.2); /*(0,4)*/
      } else if (leftJoystick.getRawButton(3)) {
        robotHardware.moveBackward(.2); /*(0,4)*/
      } else if (leftJoystick.getRawButton(4)) {
        robotHardware.turnLeftInplace(.2);
      } else if (leftJoystick.getRawButton(5)) {
        robotHardware.turnRightInplace(.2);
      } else {
        double driveJoyStickX = leftJoystick.getX();
        double driveJoyStickY = -leftJoystick.getY();

        double drivePowerLeft = driveJoyStickY + driveJoyStickX;
        double drivePowerRight = driveJoyStickY - driveJoyStickX;

        robotHardware.setDrivePower(drivePowerLeft, drivePowerRight);
      }
    }

    @Override
    public void testInit () {
    }

    @Override
    public void testPeriodic () {
    }


}
