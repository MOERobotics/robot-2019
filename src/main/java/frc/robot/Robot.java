/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.genericrobot.SuperMOEva;
import frc.robot.genericrobot.MOErio;

import javax.sound.sampled.Port;
import java.util.Arrays;
import java.util.Date;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;
import edu.wpi.first.cameraserver.CameraServer.*;

public class Robot extends TimedRobot {

	//private SuperMOEva     robotHardware = new SuperMOEva();
    private MOErio         robotHardware = new MOErio();
	private Joystick       leftJoystick  = new Joystick(0);
	private XboxController functionStick = new XboxController(1);
	private GenericAuto    autoProgram   = new MOErioCargoFrontAutoBonus();

	//lidar
	SerialPort Blinky;
	boolean PortOpen = false;

	//drive elevator
	static final double upperElevator = 1;
	static final double bottomElevator = -0.6;

	/* kP = 0.1, kI = 8*10^-3, kD = 0.0*/


	@Override
	public void robotInit() {
		autoProgram.robot = robotHardware;

		/* Launch the camera server. */
		CameraServer.getInstance().startAutomaticCapture();

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

		Lidar.init(Blinky);
	}

	@Override
	public void robotPeriodic () {
		SmartDashboard.putString ("Robot Class"        , robotHardware.getClass().getSimpleName() );
		SmartDashboard.putString ("Auto Class"         , autoProgram.getClass().getName()   );
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
		SmartDashboard.putNumber ("Climber Power: "    , robotHardware.getClimbPower()            );

		SmartDashboard.putBoolean("Is ArmDown"         , robotHardware.isArmDown()                );
		SmartDashboard.putBoolean("Is ArmUp"           , robotHardware.isArmUp()                  );
		SmartDashboard.putBoolean("Is Elevator Down"   , robotHardware.isElevatorDown()           );
		SmartDashboard.putBoolean("Is Elevator Up"     , robotHardware.isElevatorUp()             );
		SmartDashboard.putBoolean("Is Turret Left"     , robotHardware.isTurretLeft()             );
		SmartDashboard.putBoolean("Is Turret Right"    , robotHardware.isTurretRight()            );
		SmartDashboard.putBoolean("SAFETY MOEVERRIDE"  , robotHardware.getSafetyOverride()        );

		SmartDashboard.putString("Shifter State: ", robotHardware.getShifterSolenoidState().name());
		SmartDashboard.putBoolean("Spear State: ", robotHardware.getSpearShaftState());
		SmartDashboard.putBoolean("Hatch Grabber State: ", robotHardware.getSpearHookState());

		String modified = "fail";
				try {
					modified = new Date(
							Robot
								.class
								.getResource("Main.class")
								.openConnection()
								.getLastModified()
					).toString();
				} catch (Exception ignored) {}
				SmartDashboard.putString("modifiedDate: ", modified);

		SmartDashboard.putNumber("autostep: "         , autoProgram.autoStep                   );
		autoProgram.printSmartDashboard();


		Lidar.getLidar(robotHardware, Blinky);

		if (leftJoystick.getRawButtonPressed (13)) robotHardware.setOffsets();
		if (leftJoystick.getRawButtonReleased(13)) robotHardware.clearOffsets();
		if (leftJoystick.getRawButtonPressed (14)) robotHardware.setSafetyOverride(true);
		if (leftJoystick.getRawButtonReleased(14)) robotHardware.setSafetyOverride(false);
	}

	@Override
	public void disabledInit () {
		robotHardware.shiftSpearHook(false);
		robotHardware.shiftSpearShaft(false);

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
		//Driving Adjustments
		     if (leftJoystick.getTrigger()   )  robotHardware.moveForward     (.25);
		else if (leftJoystick.getRawButton(3))  robotHardware.moveBackward    (.25);
		else if (leftJoystick.getRawButton(2))  robotHardware.turnLeftInplace (.25);
		else if (leftJoystick.getRawButton(4))  robotHardware.turnRightInplace(.25);

		//Individual motors (For testing)
		//else if (leftJoystick.getRawButton(5))  robotHardware.driveSA(0.5);
		//else if (leftJoystick.getRawButton(6))  robotHardware.driveSB(0.5);
		//else if (leftJoystick.getRawButton(7))  robotHardware.driveFA(0.5);
		//else if (leftJoystick.getRawButton(8))  robotHardware.driveFB(0.5);

		//Manual Control
		else {
			double driveJoyStickX =  leftJoystick.getX();
			double driveJoyStickY = -leftJoystick.getY();

			if (Math.abs(driveJoyStickY) < 0.05) driveJoyStickY = 0.0;
			else if (driveJoyStickX > 0) driveJoyStickX -= 0.05;
			else if (driveJoyStickX < 0) driveJoyStickX += 0.05;
			//Attempt to drive straight if joystick is within 15% of vertical
			if (Math.abs(driveJoyStickX) < 0.15) driveJoyStickX = 0.0;
			else if (driveJoyStickX > 0) driveJoyStickX -= 0.15;
			else if (driveJoyStickX < 0) driveJoyStickX += 0.15;
			driveJoyStickY *= 1.052631579;
			driveJoyStickX *= .85;

			double drivePowerLeft  = driveJoyStickY + driveJoyStickX;
			double drivePowerRight = driveJoyStickY - driveJoyStickX;

			robotHardware.setDrivePower(drivePowerLeft, drivePowerRight);
		}

		//Climbing

		if      (leftJoystick.getRawButton( 9)) robotHardware.climbUp  (1.0);
		else if (leftJoystick.getRawButton(10)) robotHardware.climbDown(0.3);
		else                                    robotHardware.climb    (0.0);


		/*
		if      (leftJoystick.getRawButton(7)) robotHardware.climb2(true);
		else if (leftJoystick.getRawButton(8)) robotHardware.climb2(false);
        */
		//25084 tk 59.4 in

		//Shifting

		if (leftJoystick.getRawButtonPressed (12)) robotHardware.shiftLow();
		if (leftJoystick.getRawButtonReleased(12)) robotHardware.shiftHigh ();

		//hatchGrab
		if      (functionStick.getXButton()) robotHardware.spearIn    ();
		else if (functionStick.getYButton()) robotHardware.spearOut   ();
		if      (functionStick.getAButton()) robotHardware.spearHook  ();
		else if (functionStick.getBButton()) robotHardware.spearUnhook();

		//roller
		//TODO: bumpers
		if      (functionStick.getBumper(Hand.kLeft )) robotHardware.rollIn (0.8);
		else if (functionStick.getBumper(Hand.kRight)) robotHardware.rollOut(0.5);
		else                                           robotHardware.driveRoller(0.0);




		double armPower    = functionStick.getY(Hand.kRight);
		double turretPower = functionStick.getX(Hand.kRight);

		//arm
		//Right stick, up/down rotates.
		if (
			Math.abs(armPower) < 0.3 ||
			Math.abs(armPower) < Math.abs(turretPower)
		) armPower = 0;
		else if (armPower > 0) armPower -= 0.3;
		else if (armPower < 0) armPower += 0.3;
		robotHardware.driveArm(-armPower*0.5);

		//turret
		//Right stick, left/right rotates.
		if (
			Math.abs(turretPower) < 0.3 ||
			Math.abs(turretPower) < Math.abs(armPower)
		) turretPower = 0;
		else if (turretPower > 0) turretPower -= 0.3;
		else if (turretPower < 0) turretPower += 0.3;
		robotHardware.driveTurret(turretPower*0.5);

		//elevator
		double elevatorPower =
			functionStick.getTriggerAxis(Hand.kRight) -
			functionStick.getTriggerAxis(Hand.kLeft );

		if (
				Math.abs(elevatorPower) < 0.3
		) elevatorPower = 0;
		else if (elevatorPower > 0) elevatorPower -= 0.3;
		else if (elevatorPower < 0) elevatorPower += 0.3;
		robotHardware.driveElevator(elevatorPower*0.8);

		POVDirection controlPadDirection = POVDirection.getDirection(functionStick.getPOV());
		switch (controlPadDirection) {
			case EAST:
			case WEST:
			case NORTHWEST:
			case NORTH:
			case NORTHEAST:
			case SOUTHWEST:
			case SOUTH:
			case SOUTHEAST:
			default:
				break;
		}


	}

	public enum POVDirection {
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
		POVDirection(int angle) { this.angle = angle; }
		public int getAngle() { return angle; }


		//Kevin voodoo to turn ints into directions
		public static final Map<Integer, POVDirection> directionMap =
			Arrays.stream(POVDirection.values()).collect(
				Collectors.toMap(
					POVDirection::getAngle,
					Function.identity()
				)
			);
		public static POVDirection getDirection(int angle) {
			return directionMap.getOrDefault(angle, POVDirection.NULL);
		}
	}

	@Override
	public void testInit () {
	}

	@Override
	public void testPeriodic () {
	}

}
