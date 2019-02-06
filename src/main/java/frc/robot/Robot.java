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

    GenericAuto autoProgram = new DriveStraightAuto();

    public static int numSensors = 2;
    public static int[] lidar = new int[numSensors];

    //lidar
    SerialPort Blinky;
    boolean PortOpen = false;

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
        public void robotPeriodic() {
            SmartDashboard.putNumber("Yaw: ", robotHardware.getHeadingDegrees());
            SmartDashboard.putNumber("Left Encoder: ", robotHardware.getDistanceLeftInches());
            SmartDashboard.putNumber("Right Encoder: ", robotHardware.getDistanceRightInches());
            SmartDashboard.putNumber("Left Drive Power: ", robotHardware.getLeftDrivePower());
            SmartDashboard.putNumber("Right Drive Power: ", robotHardware.getRightDrivePower());
            SmartDashboard.putNumber("autostep: ", autoProgram.autoStep);

        }

        @Override
        public void disabledInit() {
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
        public void autonomousPeriodic() {
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

        }

        @Override
        public void testInit () {
        }

        @Override
        public void testPeriodic () {
        }


    }

