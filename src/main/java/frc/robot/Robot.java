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
import io.github.pseudoresonance.pixy2api*;
import io.github.pseudoresonance.pixy2api.links.SPILink;

import java.util.function.BooleanSupplier;
import java.util.function.Predicate;

public class Robot extends TimedRobot {

	private GenericRobot   robotHardware = new SuperMOEva();
	private Joystick       leftJoystick  = new Joystick(0);
	private XboxController functionStick = new XboxController(1);
	private GenericAuto    autoProgram   = new DriveStraightAuto();

	//lidar
	//SerialPort Blinky;
	// boolean PortOpen = false;
	// public static int numSensors = 8;
	// public static int[] lidar = new int[numSensors];

	//drive elevator
	static final double upperElevator = 1;
	static final double bottomElevator = -0.6;

	/* kP = 0.1, kI = 8*10^-3, kD = 0.0*/

    //pixy line-detection camera
    private SPILink pixySPI = new SPILink();
    private Pixy2 pixyCam = Pixy2.createInstance(pixySPI);
    private long updateNum;


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
		SmartDashboard.putNumber("Yaw: "              , robotHardware.getHeadingDegrees()      );
		SmartDashboard.putNumber("Left Encoder: "     , robotHardware.getDistanceLeftInches()  );
		SmartDashboard.putNumber("Right Encoder: "    , robotHardware.getDistanceRightInches() );
		SmartDashboard.putNumber("Left Drive Power: " , robotHardware.getLeftDrivePower()      );
		SmartDashboard.putNumber("Right Drive Power: ", robotHardware.getRightDrivePower()     );
		SmartDashboard.putNumber("Turret Encoder: "   , robotHardware.getTurretPower()         );
		SmartDashboard.putNumber("Elevator Encoder: " , robotHardware.getElevatorPower()       );
		SmartDashboard.putNumber("Arm Power: "        , robotHardware.getArmPower()            );
		SmartDashboard.putNumber("Roller Power: "     , robotHardware.getRollerPower()         );
		SmartDashboard.putNumber("autostep: "         , autoProgram.autoStep                   );
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
	}

	@Override
	public void autonomousPeriodic () {
		robotHardware.checkSafety();
		autoProgram.run();
	}

	@Override
	public void teleopInit () {
	}

	@Override
	public void teleopPeriodic () {
		//Driving
		if      (leftJoystick.getRawButton(2))  robotHardware.moveForward(.2);
		else if (leftJoystick.getTrigger()   )  robotHardware.moveForward(.2);
		else if (leftJoystick.getRawButton(3))  robotHardware.moveBackward(.2);
		else if (leftJoystick.getRawButton(4))  robotHardware.turnLeftInplace(.2);
		else if (leftJoystick.getRawButton(5))  robotHardware.turnRightInplace(.2);

		//Individual motors (For testing)
		else if (leftJoystick.getRawButton(6))  robotHardware.driveSA(0.5);
		else if (leftJoystick.getRawButton(7))  robotHardware.driveSB(0.5);
		else if (leftJoystick.getRawButton(8))  robotHardware.driveFA(0.5);
		else if (leftJoystick.getRawButton(9))  robotHardware.driveFB(0.5);

		//Manual Control
		else {
			double driveJoyStickX =  leftJoystick.getX();
			double driveJoyStickY = -leftJoystick.getY();

			//Attempt to drive straight if joystick is within 15% of vertical
			if (Math.abs(driveJoyStickX) < 0.15) driveJoyStickX = 0.0;

			double drivePowerLeft  = driveJoyStickY + driveJoyStickX;
			double drivePowerRight = driveJoyStickY - driveJoyStickX;

			robotHardware.setDrivePower(drivePowerLeft, drivePowerRight);
		}

		//roller
		if      (functionStick.getAButton()) robotHardware.rollIn (0.3);
		else if (functionStick.getBButton()) robotHardware.rollOut(0.3);
		else                                 robotHardware.driveRoller(0);


		//arm
		if      (functionStick.getBumper(Hand.kLeft )) robotHardware.driveArm( 0.8);
		else if (functionStick.getBumper(Hand.kRight)) robotHardware.driveArm(-0.4);
		else                                           robotHardware.driveArm( 0.0);

		//turret
		if      (functionStick.getStickButton(Hand.kRight)) robotHardware.driveTurret( 0.5);
		else if (functionStick.getStickButton(Hand.kLeft )) robotHardware.driveTurret(-0.5);
		else                                                robotHardware.driveTurret( 0.0);

		//elevator
		if      (functionStick.getTriggerAxis(Hand.kLeft ) > 0.3) robotHardware.driveElevator(-0.6 * functionStick.getTriggerAxis(Hand.kLeft ));
		else if (functionStick.getTriggerAxis(Hand.kRight) > 0.3) robotHardware.driveElevator( 1.0 * functionStick.getTriggerAxis(Hand.kRight));
		else                                                      robotHardware.driveElevator( 0.0);

		//pixycam
        pixyCam.getLine().getAllFeatures();
        Pixy2Line.Vector[] vec = pixyCam.getLine().getVectors();

        if(vec == null){ return; }
        //Print vector coords to smartdashboard
        SmartDashboard.putNumber(vec[0].getX0());
        SmartDashboard.putNumber(vec[0].getX1());
        SmartDashboard.putNumber(vec[0].getY0());
        SmartDashboard.putNumber(vec[0].getX1());

        for(int i=0;i<vec.length;i++){
            System.out.println(vec[i].toString());
        }

	}

	@Override
	public void testInit () {
	}

	@Override
	public void testPeriodic () {
	}

}
