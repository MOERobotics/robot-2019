/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.genericrobot.SuperMOEva;
//import io.github.pseudoresonance.pixy2api.Pixy2Line;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.UsbCamera;

import javax.sound.sampled.Port;
import java.util.Arrays;

import java.util.Date;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;

public class Robot extends TimedRobot {

	private GenericRobot   robotHardware = new SuperMOEva();
	private Joystick       leftJoystick  = new Joystick(0);
	private XboxController functionStick = new XboxController(1);
	private GenericAuto    autoProgram   = new UnitTestElevArmPos();
//	UsbCamera cam1;
    int smartDashCounter = 0;


	public PixyCam pixy = new PixyCam() {{
		init();
		run();
		start();
	}};

	//lidar
	SerialPort Blinky;
	boolean PortOpen = false;

	//drive elevator
	static final double upperElevator = 1;
	static final double bottomElevator = -0.6;

	/* kP = 0.1, kI = 8*10^-3, kD = 0.0*/

	long startTime;
	int grabStep;

	public double ClimbEncoderOrigin = 0;
	static final double HABheight = 20;

	@Override
	public void robotInit() {
		autoProgram.robot = robotHardware;
		autoProgram.LeftSide = 1;
		robotHardware.enableElevatorLimits(true); //-Brian
		robotHardware.enableArmLimits(true); //-Brian
		robotHardware.shiftLow();
		robotHardware.floorPickupUp();

		ClimbEncoderOrigin = robotHardware.getClimberLEncoderCount();
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

		if (PortOpen) Lidar.init(Blinky);
		CameraServer.getInstance().startAutomaticCapture();
	}

	@Override
	public void robotPeriodic () {
	    if (0==(smartDashCounter++ % 10)) { //-Brian
            SmartDashboard.putString("Robot Class", robotHardware.getClass().getSimpleName());
            SmartDashboard.putString("Auto Class", autoProgram.getClass().getSimpleName());

            SmartDashboard.putNumber("Yaw: ", robotHardware.getHeadingDegrees());
            SmartDashboard.putNumber("Roll: ", robotHardware.getRollDegrees());
            SmartDashboard.putNumber("Pitch: ", robotHardware.getPitchDegrees());
            SmartDashboard.putNumber("Left Encoder: ", robotHardware.getDistanceLeftInches());
            SmartDashboard.putNumber("Right Encoder: ", robotHardware.getDistanceRightInches());
            SmartDashboard.putNumber("Arm Encoder: ", robotHardware.getArmEncoderCount());
            SmartDashboard.putNumber("Elevator Encoder: ", robotHardware.getElevatorEncoderCount());

            SmartDashboard.putNumber("Left Drive Power: ", robotHardware.getLeftDrivePower());
            SmartDashboard.putNumber("Right Drive Power: ", robotHardware.getRightDrivePower());
            SmartDashboard.putNumber("Arm Power: ", robotHardware.getArmPower());
            SmartDashboard.putNumber("Elevator Power: ", robotHardware.getElevatorPower());
            SmartDashboard.putNumber("Roller Power: ", robotHardware.getRollerPower());
            SmartDashboard.putNumber("Climber Power: ", robotHardware.getClimbPower());

            SmartDashboard.putBoolean("Is ArmDown: ", robotHardware.isArmDown());
            SmartDashboard.putBoolean("Is ArmUp: ", robotHardware.isArmUp());
            SmartDashboard.putBoolean("Is Elevator Down: ", robotHardware.isElevatorDown());
            SmartDashboard.putBoolean("Is Elevator Up: ", robotHardware.isElevatorUp());
            SmartDashboard.putBoolean("SAFETY MOEVERRIDE", robotHardware.getSafetyOverride());

            SmartDashboard.putString("Shifter State: ", robotHardware.getShifterSolenoidState().name());
            SmartDashboard.putBoolean("Spear State: ", robotHardware.getSpearShaftState());
            SmartDashboard.putBoolean("Hatch Grabber State: ", robotHardware.getSpearHookState());
            SmartDashboard.putBoolean("Floor Pickup State: ", robotHardware.getFloorPickupState());

            SmartDashboard.putBoolean("Elevator Forward Limit Enabled: ", robotHardware.isElevForwardLimitEnabled());
            SmartDashboard.putBoolean("At Elevator Forward Limit: ", robotHardware.atElevForwardLimit());
            SmartDashboard.putBoolean("Elevator Reverse Limit Enabled: ", robotHardware.isElevReverseLimitEnabled());
            SmartDashboard.putBoolean("At Elevator Reverse Limit: ", robotHardware.atElevReverseLimit());

            SmartDashboard.putBoolean("Arm Forward Limit Enabled: ", robotHardware.isArmForwardLimitEnabled());
            SmartDashboard.putBoolean("At Arm Forward Limit: ", robotHardware.atArmForwardLimit());
            SmartDashboard.putBoolean("Arm Reverse Limit Enabled: ", robotHardware.isArmReverseLimitEnabled());
            SmartDashboard.putBoolean("At Arm Reverse Limit: ", robotHardware.atArmReverseLimit());

            SmartDashboard.putNumber("Frogger Power: ", robotHardware.getClimbPower());
            SmartDashboard.putNumber("Frogger Left Encoder: ", robotHardware.getClimberLEncoderCount());
            SmartDashboard.putNumber("Frogger Right Encoder: ", robotHardware.getClimberREncoderCount());

            String modified = "fail";
            try {
                modified = new Date(
                        Robot
                                .class
                                .getResource("Main.class")
                                .openConnection()
                                .getLastModified()
                ).toString();
            } catch (Exception ignored) {
            }
            SmartDashboard.putString("modifiedDate: ", modified);

            SmartDashboard.putNumber("autostep: ", autoProgram.autoStep);
            SmartDashboard.putNumber("RightSide: ", autoProgram.LeftSide);
            autoProgram.printSmartDashboard();

            SmartDashboard.putString("PixyInfo: ", pixy.toString());
        }
		if (leftJoystick.getRawButtonPressed (13)) robotHardware.setOffsets();
		if (leftJoystick.getRawButtonReleased(13)) robotHardware.clearOffsets();
		if (leftJoystick.getRawButtonPressed (14)) robotHardware.setSafetyOverride(true);
		if (leftJoystick.getRawButtonReleased(14)) robotHardware.setSafetyOverride(false);

		if (PortOpen) Lidar.getLidar(robotHardware, Blinky);
	}

	@Override
	public void disabledInit () {
		robotHardware.shiftSpearHook(false);
		robotHardware.shiftSpearShaft(false);

	}

	@Override
	public void disabledPeriodic () {
//        ClimbEncoderOrigin = robotHardware.getClimberLEncoderCount();

        if (leftJoystick.getRawButton(2)) {
			robotHardware.resetYaw();
			robotHardware.resetDriveEncoders();
		}
		if (leftJoystick.getRawButton(5)){
			autoProgram = new MOErioCargoFrontAuto();
			autoProgram.LeftSide = 1;
			autoProgram.robot = robotHardware;
		} else if (leftJoystick.getRawButton(6)){
			autoProgram = new MOErioCargoFrontAuto();
			autoProgram.LeftSide = -1;
			autoProgram.robot = robotHardware;
		} else if (leftJoystick.getRawButton(7)){
			autoProgram = new MOErioCargoFrontAutoBonus();
			autoProgram.LeftSide = 1;
			autoProgram.robot = robotHardware;
		} else if (leftJoystick.getRawButton(8)){
			autoProgram = new MOErioCargoFrontAutoBonus();
			autoProgram.LeftSide = -1;
			autoProgram.robot = robotHardware;
		} else if (leftJoystick.getRawButton(9)){
			autoProgram = new MOErioCargoSideAutoBonus();
			autoProgram.LeftSide = 1;
			autoProgram.robot = robotHardware;
		} else if (leftJoystick.getRawButton(10)){
			autoProgram = new MOErioCargoSideAutoBonus();
			autoProgram.LeftSide = -1;
			autoProgram.robot = robotHardware;
		}

		if (functionStick.getAButton()) robotHardware.enableElevatorLimits(false);
		else if (functionStick.getBButton()) robotHardware.enableArmLimits(false);
		else if (functionStick.getXButton()) robotHardware.enableElevatorLimits(true);
		else if (functionStick.getYButton()) robotHardware.enableArmLimits(true);

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
		//robotHardware.enableElevatorLimits(true);
		//robotHardware.enableArmLimits(true);
		grabStep = 0;
	}

	@Override
	public void teleopPeriodic () {
		//Driving Adjustments
		if (leftJoystick.getTrigger()   )  robotHardware.moveForward     (.25);
		else if (leftJoystick.getRawButton(2))  robotHardware.turnLeftInplace (.25);
		else if (leftJoystick.getRawButton(3))  robotHardware.moveBackward    (.25);
		else if (leftJoystick.getRawButton(4))  robotHardware.turnRightInplace(.25);

		//Individual motors (For testing)
		//else if (leftJoystick.getRawButton(5))  robotHardware.driveSA(0.5);
		//else if (leftJoystick.getRawButton(6))  robotHardware.driveSB(0.5);
		//else if (leftJoystick.getRawButton(7))  robotHardware.driveFA(0.5);
		//else if (leftJoystick.getRawButton(8))  robotHardware.driveFB(0.5);

		 else if (leftJoystick.getRawButton(5)) robotHardware.enableElevatorLimits(false);
		 else if (leftJoystick.getRawButton(6)) robotHardware.enableArmLimits(false);
		 else if (leftJoystick.getRawButton(7)) robotHardware.enableElevatorLimits(true);
		 else if (leftJoystick.getRawButton(8)) robotHardware.enableArmLimits(true);


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

        //POVDirection controlPadDirection = POVDirection.getDirection(functionStick.getPOV());
		if (Math.abs(robotHardware.getClimberLEncoderCount()-ClimbEncoderOrigin) < HABheight) {
            if      (leftJoystick.getRawButton( 9)) robotHardware.climb  (0.5);
            else if (leftJoystick.getRawButton(10)) robotHardware.climb(-1.0);


                //else if (controlPadDirection == POVDirection.EAST)   robotHardware.climbFreeUp(0.3);
                //else if (controlPadDirection == POVDirection.WEST)   robotHardware.climbSupportUp(0.3);

            else                                    robotHardware.climb    (0.0);
        }
        if (Math.abs(robotHardware.getClimberLEncoderCount()-ClimbEncoderOrigin) >= HABheight) {
            if (leftJoystick.getRawButton(10))
            {
                robotHardware.climb2(true);
            }
        }


		//if      (leftJoystick.getRawButton(7)) robotHardware.climb2(true);
		//else if (leftJoystick.getRawButton(8)) robotHardware.climb2(false);

		//Shifting
        if(leftJoystick.getRawButtonPressed(11))    robotHardware.climb2(true);
        if(leftJoystick.getRawButtonReleased(11))    robotHardware.climb2(false);

        if (leftJoystick.getRawButtonPressed (12)) robotHardware.shiftHigh();
		if (leftJoystick.getRawButtonReleased(12)) robotHardware.shiftLow ();

		//hatchGrab
		if      (functionStick.getBButton()) robotHardware.spearIn    ();
		else if (functionStick.getAButton()) robotHardware.spearOut   ();
		if      (functionStick.getXButton()) robotHardware.spearHook  ();
		else if (functionStick.getYButton()) robotHardware.spearUnhook();

		//roller
		//TODO: bumpers
		if      (functionStick.getBumper(Hand.kLeft )) robotHardware.rollOut (0.5);
		else if (functionStick.getBumper(Hand.kRight)) robotHardware.rollIn(0.8);
		else                                           robotHardware.driveRoller(0.0);


		double armPower    = functionStick.getY(Hand.kRight);
		/*if(functionStick.getY(Hand.kRight) != 0) armPower = functionStick.getY(Hand.kRight);
		else armPower = 0.4;*/

		//arm
		//Right stick, up/down rotates.
		if (
			Math.abs(armPower) < 0.2
		) armPower = 0;
		else if (armPower > 0) armPower -= 0.2;
		else if (armPower < 0) armPower += 0.2;
		robotHardware.driveArm(-armPower);


		/*else if (armPower > 0) armPower -= 0.2;
		else if (armPower < 0) armPower += 0.2;*/
		//robotHardware.driveArm(-armPower*0.5);
		/*else if (armPower > 0) armPower = 0.4;
		else if (armPower < 0) armPower = -0.4;*/



		//elevator
		//elevator position = -29.7
		double elevatorPower =
			functionStick.getTriggerAxis(Hand.kRight) -
			functionStick.getTriggerAxis(Hand.kLeft );

		if (
				Math.abs(elevatorPower) < 0.3
		) elevatorPower = 0;
		else if (elevatorPower > 0) elevatorPower -= 0.3;
		else if (elevatorPower < 0) elevatorPower += 0.3;
		robotHardware.driveElevator(elevatorPower*0.8);

		if (functionStick.getStickButton(Hand.kLeft)) {
			if (robotHardware.atElevForwardLimit()) {
				robotHardware.setElevatorOrigin(robotHardware.getElevatorEncoderCount());
			} else {
				robotHardware.driveElevator(0.3);
			}
			SmartDashboard.putNumber("Elevator Origin: ", robotHardware.getElevatorOrigin());

			if (robotHardware.atArmForwardLimit()) {
				robotHardware.setArmOrigin(robotHardware.getArmEncoderCount());
			} else {
				robotHardware.driveArm(0.3);
			}
			SmartDashboard.putNumber("Arm Origin: ", robotHardware.getArmOrigin());
		}

		POVDirection controlPadDirection = POVDirection.getDirection(functionStick.getPOV());
		switch (controlPadDirection) {
			case NORTH:
				robotHardware.floorPickupUp();
				break;
			case SOUTH:
				robotHardware.floorPickupDown();
				break;
			case EAST:
				robotHardware.climbFreeUp(0.3);
				break;
			case WEST:
				robotHardware.climbSupportUp(0.3);
				break;
			case NORTHWEST:
			case SOUTHEAST:
			case NORTHEAST:
			case SOUTHWEST:
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
