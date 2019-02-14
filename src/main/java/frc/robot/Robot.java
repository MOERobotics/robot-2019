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
//import frc.robot.genericrobot.CaMOElot;
//import frc.robot.genericrobot.MOErio;

//import java.util.function.BooleanSupplier;
//import java.util.function.Predicate;

public class Robot extends TimedRobot {

	private GenericRobot   robotHardware = new SuperMOEva();
	Joystick       leftJoystick  = new Joystick(1);
	XboxController functionStick = new XboxController(2);
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
		SmartDashboard.putNumber("Pitch: ", robotHardware.getPitchDegrees());
		SmartDashboard.putNumber("Roll: ", robotHardware.getRollDegrees());
		SmartDashboard.putNumber("Left Encoder: "     , robotHardware.getDistanceLeftInches()  );
		SmartDashboard.putNumber("Right Encoder: "    , robotHardware.getDistanceRightInches() );
		SmartDashboard.putNumber("Left Drive Power: " , robotHardware.getLeftDrivePower()      );
		SmartDashboard.putNumber("Right Drive Power: ", robotHardware.getRightDrivePower()     );

		SmartDashboard.putNumber("Arm Encoder: "      , robotHardware.getArmEncoderCount()     );
		SmartDashboard.putNumber("Turret Encoder: "   , robotHardware.getTurretEncoderCount()  );
		SmartDashboard.putNumber("Elevator Encoder: " , robotHardware.getElevatorEncoderCount());

		SmartDashboard.putNumber("Arm Power: "        , robotHardware.getArmPower()            );
		SmartDashboard.putNumber("Turret Power: "     , robotHardware.getTurretPower()  );
		SmartDashboard.putNumber("Elevator Power: "   , robotHardware.getElevatorPower());
		SmartDashboard.putNumber("Roller Power: "     , robotHardware.getRollerPower()         );

		SmartDashboard.putString("Shifter State: ", robotHardware.getShifterSolenoidState().name());
		SmartDashboard.putBoolean("Spear State: ", robotHardware.getSpearSolenoidState());
		SmartDashboard.putBoolean("Hatch Grabber State: ", robotHardware.getHatchGrabbSolenoidState());

		SmartDashboard.putBoolean("Is Turret Left: ", robotHardware.isTurretLeft());
		SmartDashboard.putBoolean("Is Turret Right: ", robotHardware.isTurretRight());
		SmartDashboard.putBoolean("Is Arm Down: ", robotHardware.isArmDown());
		SmartDashboard.putBoolean("Is Arm Up: ", robotHardware.isArmUp());
		SmartDashboard.putBoolean("Is Elevator Down: ", robotHardware.isElevatorDown());
		SmartDashboard.putBoolean("Is Elevator Up: ", robotHardware.isElevatorUp());

		SmartDashboard.putNumber("autostep: "         , autoProgram.autoStep                   );
		autoProgram.printSmartDashboard();

		//robotHardware.checkSafety();
	}

	@Override
	public void disabledInit () {
	}

	@Override
	public void disabledPeriodic () {
		if (leftJoystick.getRawButton(2)) {
			robotHardware.resetYaw();
			//robotHardware.resetDriveEncoders();
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

		else if (leftJoystick.getRawButton(11)) robotHardware.shiftHigh();
		else if (leftJoystick.getRawButton(12))	robotHardware.shiftLow();

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

		//hatchGrab
		if (functionStick.getXButton()) robotHardware.spearIn();
		else if (functionStick.getYButton()) robotHardware.spearOut();
		if      (functionStick.getAButton()) robotHardware.hatchGrabIn();
		else if (functionStick.getBButton()) robotHardware.hatchGrabOut();

		//roller
		//controls all set to the left stick. up to roll in, down to roll out.
		if (functionStick.getY(Hand.kLeft) > 0.1) robotHardware.rollIn(Math.abs(functionStick.getY(Hand.kLeft)) * 0.3);
		else if (functionStick.getY(Hand.kLeft) < -0.1) robotHardware.rollOut(Math.abs(functionStick.getY(Hand.kLeft)) * 0.3);
		else robotHardware.rollIn(0);

		//arm
		if      (functionStick.getBumper(Hand.kLeft )) robotHardware.driveArm( 0.3);
		else if (functionStick.getBumper(Hand.kRight)) robotHardware.driveArm(-0.3);
		else                                           robotHardware.driveArm( 0.0);

		//turret
		//controls all set to the right stick. up to move right, down to move left.
		if (functionStick.getY(Hand.kRight) > 0.1) robotHardware.driveTurret(0.3 * functionStick.getY(Hand.kRight));
		else if (functionStick.getY(Hand.kRight) < 0.1) robotHardware.driveTurret(0.3 * functionStick.getY(Hand.kRight));
		else robotHardware.driveTurret(0);

		/*
		//set these buttons to something else! (if used)

		if      (functionStick.getRawButton(8)) robotHardware.driveTurret( 0.3);
		else if (functionStick.getRawButton(9)) robotHardware.driveTurret(-0.3);
		else                                 robotHardware.driveTurret( 0.0);*/

		//elevator
		if      (functionStick.getTriggerAxis(Hand.kLeft ) > 0.3) robotHardware.driveElevator(-0.3 * functionStick.getTriggerAxis(Hand.kLeft ));
		else if (functionStick.getTriggerAxis(Hand.kRight) > 0.3) robotHardware.driveElevator( 0.3 * functionStick.getTriggerAxis(Hand.kRight));
		else                                                      robotHardware.driveElevator( 0.0);
	}

	@Override
	public void testInit () {
	}

	@Override
	public void testPeriodic () {
	}

}
