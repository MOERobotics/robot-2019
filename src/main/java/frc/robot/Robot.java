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

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;

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

	public static final Map<Integer, POVDirection> directionMap =
		Arrays.stream(POVDirection.values()).collect(
			Collectors.toMap(
				(POVDirection x) -> x.getAngle(),
				(POVDirection x) -> x
			)
		);

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
		SmartDashboard.putString ("Robot Class"        , robotHardware.getClass().getSimpleName() );
		SmartDashboard.putString ("Auto Class"         , autoProgram.getClass().getSimpleName()   );

		SmartDashboard.putNumber ("Yaw: "              , robotHardware.getHeadingDegrees()        );
		SmartDashboard.putNumber ("Roll: "             , robotHardware.getRollDegrees()           );
		SmartDashboard.putNumber ("Pitch: "            , robotHardware.getPitchDegrees()          );
		SmartDashboard.putNumber ("Left Encoder: "     , robotHardware.getDistanceLeftInches()    );
		SmartDashboard.putNumber ("Right Encoder: "    , robotHardware.getDistanceRightInches()   );
		SmartDashboard.putNumber ("Arm Encoder: "      , robotHardware.getArmEncoderCount()       );
		SmartDashboard.putNumber ("Turret Encoder: "   , robotHardware.getTurretEncoderCount()    );
		SmartDashboard.putNumber ("Elevator Encoder: " , robotHardware.getElevatorEncoderCount()  );

		SmartDashboard.putNumber ("Left Drive Power: " , robotHardware.getLeftDrivePower()        );
		SmartDashboard.putNumber ("Right Drive Power: ", robotHardware.getRightDrivePower()       );
		SmartDashboard.putNumber ("Arm Power: "        , robotHardware.getArmPower()              );
		SmartDashboard.putNumber ("Turret Power: "     , robotHardware.getTurretPower()           );
		SmartDashboard.putNumber ("Elevator Power: "   , robotHardware.getElevatorPower()         );
		SmartDashboard.putNumber ("Roller Power: "     , robotHardware.getRollerPower()           );

		SmartDashboard.putBoolean("Is ArmDown"         , robotHardware.isArmDown()                );
		SmartDashboard.putBoolean("Is ArmUp"           , robotHardware.isArmUp()                  );
		SmartDashboard.putBoolean("Is Elevator Down"   , robotHardware.isElevatorDown()           );
		SmartDashboard.putBoolean("Is Elevator Up"     , robotHardware.isElevatorUp()             );
		SmartDashboard.putBoolean("Is Turret Left"     , robotHardware.isTurretLeft()             );
		SmartDashboard.putBoolean("Is Turret Right"    , robotHardware.isTurretRight()            );

		SmartDashboard.putString("Shifter State: ", robotHardware.getShifterSolenoidState().name());
		SmartDashboard.putBoolean("Spear State: ", robotHardware.getSpearShaftState());
		SmartDashboard.putBoolean("Hatch Grabber State: ", robotHardware.getSpearHookState());

		SmartDashboard.putNumber("autostep: "         , autoProgram.autoStep                   );
		autoProgram.printSmartDashboard();
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
		     if (leftJoystick.getTrigger()   )  robotHardware.moveForward(.2);
		else if (leftJoystick.getRawButton(2))  robotHardware.moveBackward(.2);
		else if (leftJoystick.getRawButton(3))  robotHardware.turnLeftInplace(.2);
		else if (leftJoystick.getRawButton(4))  robotHardware.turnRightInplace(.2);

		//Individual motors (For testing)
		else if (leftJoystick.getRawButton(5))  robotHardware.driveSA(0.5);
		else if (leftJoystick.getRawButton(6))  robotHardware.driveSB(0.5);
		else if (leftJoystick.getRawButton(7))  robotHardware.driveFA(0.5);
		else if (leftJoystick.getRawButton(8))  robotHardware.driveFB(0.5);

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
		if      (functionStick.getXButton()) robotHardware.spearIn();
		else if (functionStick.getYButton()) robotHardware.spearOut();
		if      (functionStick.getAButton()) robotHardware.spearHook();
		else if (functionStick.getBButton()) robotHardware.spearUnhook();

		//roller
		//controls all set to the left stick. up to roll in, down to roll out.
		double rollerPower = functionStick.getY(Hand.kLeft);
		if (Math.abs(rollerPower) < 0.1) rollerPower = 0;
		robotHardware.rollIn(rollerPower);

		//arm
		if      (functionStick.getBumper(Hand.kLeft )) robotHardware.driveArm( 0.8);
		else if (functionStick.getBumper(Hand.kRight)) robotHardware.driveArm(-0.4);
		else                                           robotHardware.driveArm( 0.0);

		//turret
		//Right stick, left/right rotates.
		double turretPower = functionStick.getX(Hand.kRight);
		if (Math.abs(turretPower) < 0.1) turretPower = 0;
		robotHardware.driveTurret(turretPower);

		//elevator
		if      (functionStick.getTriggerAxis(Hand.kLeft ) > 0.3) robotHardware.driveElevator(-0.6 * functionStick.getTriggerAxis(Hand.kLeft ));
		else if (functionStick.getTriggerAxis(Hand.kRight) > 0.3) robotHardware.driveElevator( 1.0 * functionStick.getTriggerAxis(Hand.kRight));
		else                                                      robotHardware.driveElevator( 0.0);

		//Climbing

		POVDirection controlPadDirection = directionMap.getOrDefault(functionStick.getPOV(), POVDirection.NULL);
		switch (controlPadDirection) {
			case NORTHWEST:
			case NORTH:
			case NORTHEAST:
				robotHardware.climbUp(0.3);
				break;
			case SOUTHWEST:
			case SOUTH:
			case SOUTHEAST:
				robotHardware.climbDown(0.2);
				break;
			default:
				robotHardware.climb(0);
				break;
		}

	}

	public static enum POVDirection {
		NORTH     (  0),
		NORTHEAST ( 45),
		EAST      ( 90),
		SOUTHEAST (135),
		SOUTH     (180),
		SOUTHWEST (225),
		WEST      (270), //best
		NORTHWEST (315),
		NULL      ( -1);

		private final int angle;
		private POVDirection(int angle) {
			this.angle = angle;
		}
		public int getAngle() {
			return angle;
		}
		public static int getAngle(POVDirection direction) {
			return direction.getAngle();
		}

	}

	@Override
	public void testInit () {
	}

	@Override
	public void testPeriodic () {
	}

}
