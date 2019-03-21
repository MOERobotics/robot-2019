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
import io.github.pseudoresonance.pixy2api.Pixy2Line;
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
	private Joystick 	   switchBox 	 = new Joystick(2);

	private GenericAuto    autoProgram   = new MASideAuto();
	private GenericAuto	   climbAuto 	 = new AutoFloating();

	private GenericAuto[] cargoPos = {new Cargo1(), new Cargo2(), new Cargo3()};
	private GenericAuto[] hatchPos = {new Hatch1(), new Hatch2(), new Hatch3()};
	int pos = -1;
	boolean cargo = false;
	boolean positionLock = false;


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

	//auto
    boolean autoEnable = true;
    //int startAutoStep;

	//drive elevator
	//static final double upperElevator = 1;
	//static final double bottomElevator = -0.6;

	/* kP = 0.1, kI = 8*10^-3, kD = 0.0*/

	public double ClimbEncoderOrigin = 0;
    static final double HABheight2 = 20;
	static final double HABheight3 = 20;
	boolean atHabHeight2 = false;
	boolean atHabHeight3 = false;
	private boolean footToggle = true;
	private boolean climbEnabled = false;

	private boolean functionStickDrive = false;
    double driveJoyStickX;
    double driveJoyStickY;

	int step = 1;
	double currentEncoder;

	public void noPosition() {
		pos = -1;
		positionLock = false;

		hatchPos[0].init();
		hatchPos[1].init();
		hatchPos[2].init();
		cargoPos[0].init();
		cargoPos[1].init();
		cargoPos[2].init();
	}

	@Override
	public void robotInit() {
		//System.gc();
		autoProgram.robot = robotHardware;

		climbAuto.robot = robotHardware;
		hatchPos[0].robot = robotHardware;
		hatchPos[1].robot = robotHardware;
		hatchPos[2].robot = robotHardware;
		cargoPos[0].robot = robotHardware;
		cargoPos[1].robot = robotHardware;
		cargoPos[2].robot = robotHardware;

		autoProgram.LeftSide = 1;
		robotHardware.enableElevatorLimits(false); //-Brian
		robotHardware.enableArmLimits(false); //-Brian
        robotHardware.LinearSlider(DoubleSolenoid.Value.kForward);
		robotHardware.shiftLow();

		//robotHardware.floorPickupUp();

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

		CameraServer.getInstance().startAutomaticCapture();
	}

	@Override
	public void robotPeriodic () {
        //robotHardware.checkSafety();

	    if (0==(smartDashCounter++ % 10)) { //-Brian
            SmartDashboard.putString("Robot Class", robotHardware.getClass().getSimpleName());
            SmartDashboard.putString("Auto Class", autoProgram.getClass().getSimpleName());

            SmartDashboard.putNumber("Yaw: ", robotHardware.getHeadingDegrees());
            SmartDashboard.putNumber("Roll: ", robotHardware.getRollDegrees());
            SmartDashboard.putNumber("Pitch: ", robotHardware.getPitchDegrees());
            SmartDashboard.putNumber("Left Encoder (Raw): ", robotHardware.getDistanceLeftInches() * 462);
            SmartDashboard.putNumber("Right Encoder (Raw): ", robotHardware.getDistanceRightInches() * 462);
            SmartDashboard.putNumber("Left Encoder (Inches): ", robotHardware.getDistanceLeftInches());
            SmartDashboard.putNumber("Right Encoder (Inches): ", robotHardware.getDistanceRightInches());
            SmartDashboard.putNumber("Arm Encoder: ", robotHardware.getArmEncoderCount());
            SmartDashboard.putNumber("Elevator Encoder: ", robotHardware.getElevatorEncoderCount());

            SmartDashboard.putNumber("Left Drive Power: ", robotHardware.getLeftDrivePower());
            SmartDashboard.putNumber("Right Drive Power: ", robotHardware.getRightDrivePower());
            SmartDashboard.putNumber("Arm Power: ", robotHardware.getArmPower());
            SmartDashboard.putNumber("Elevator Power: ", robotHardware.getElevatorPower());
            SmartDashboard.putNumber("Roller Power: ", robotHardware.getRollerPower());
            SmartDashboard.putNumber("Climber Power: ", robotHardware.getClimbPower());

            SmartDashboard.putBoolean("ALEX DRIVE", functionStickDrive);

            //SmartDashboard.putBoolean("Is ArmDown: ", robotHardware.isArmDown());
            //SmartDashboard.putBoolean("Is ArmUp: ", robotHardware.isArmUp());
            //SmartDashboard.putBoolean("Is Elevator Down: ", robotHardware.isElevatorDown());
            //SmartDashboard.putBoolean("Is Elevator Up: ", robotHardware.isElevatorUp());
            SmartDashboard.putBoolean("SAFETY MOEVERRIDE", robotHardware.getSafetyOverride());

            SmartDashboard.putBoolean("Shifter State: ", robotHardware.getShifterSolenoidState());
            SmartDashboard.putBoolean("Spear State: ", robotHardware.getSpearShaftState());
            SmartDashboard.putBoolean("Hatch Grabber State: ", robotHardware.getSpearHookState());
            SmartDashboard.putBoolean("Floor Pickup State: ", robotHardware.getFloorPickupState());

            //SmartDashboard.putBoolean("Elevator Forward Limit Enabled: ", robotHardware.isElevForwardLimitEnabled());
            //SmartDashboard.putBoolean("Elevator Reverse Limit Enabled: ", robotHardware.isElevReverseLimitEnabled());
            //SmartDashboard.putBoolean("Arm Forward Limit Enabled: ", robotHardware.isArmForwardLimitEnabled());
            //SmartDashboard.putBoolean("Arm Reverse Limit Enabled: ", robotHardware.isArmReverseLimitEnabled());

            SmartDashboard.putNumber("Climber Power: ", robotHardware.getClimbPower());
            SmartDashboard.putNumber("Climber Left Encoder: ", robotHardware.getClimberLEncoderCount());
            SmartDashboard.putNumber("Climber Right Encoder: ", robotHardware.getClimberREncoderCount());
			SmartDashboard.putBoolean("Foot Toggle: ", footToggle);
			SmartDashboard.putBoolean("Space State: ", robotHardware.getSpacerState());
            SmartDashboard.putString("Feet Out: ", robotHardware.getClimb2State().name());
            SmartDashboard.putBoolean("Auto Climb Enabled: ", climbEnabled);
            SmartDashboard.putString("Auto Climb: ", climbAuto.getClass().getSimpleName());

            SmartDashboard.putBoolean("At Elevator Top Limit: ", robotHardware.atElevatorTopLimit());
            SmartDashboard.putBoolean("At Elevator Bottom Limit: ", robotHardware.atElevatorBottomLimit());

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
			SmartDashboard.putBoolean("autoEnable", autoEnable);
            SmartDashboard.putNumber("RightSide: ", autoProgram.LeftSide);
            autoProgram.printSmartDashboard();

            SmartDashboard.putString("PixyInfo: ", pixy.toString());
        }
		/*if (leftJoystick.getRawButtonPressed (13)) robotHardware.setOffsets();
		if (leftJoystick.getRawButtonReleased(13)) robotHardware.clearOffsets();
		if (leftJoystick.getRawButtonPressed (14)) robotHardware.setSafetyOverride(true);
		if (leftJoystick.getRawButtonReleased(14)) robotHardware.setSafetyOverride(false);*/

		if (leftJoystick.getRawButtonPressed (11)) robotHardware.setOffsets();
		if (leftJoystick.getRawButtonReleased(11)) robotHardware.clearOffsets();
		if (leftJoystick.getThrottle() < -0.95) robotHardware.setSafetyOverride(true);
		else if (leftJoystick.getThrottle() > 0.95) robotHardware.setSafetyOverride(false);

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

        if (leftJoystick.getRawButtonPressed(2)) {
			robotHardware.resetYaw();
			robotHardware.resetDriveEncoders();
		}
		if (leftJoystick.getRawButtonPressed(3)) {
			robotHardware.resetArmPosition();
			robotHardware.resetElevatorPosition();
			robotHardware.resetClimberPosition();
		}
		if (leftJoystick.getRawButtonPressed(11)){
			autoProgram = new MAFrontAuto();
			autoProgram.LeftSide = 1;
			autoProgram.lastStep = 4;
			autoProgram.robot = robotHardware;
		} else if (leftJoystick.getRawButtonPressed(6)){
			autoProgram = new MAFrontAuto();
			autoProgram.LeftSide = -1;
			autoProgram.lastStep = 4;
			autoProgram.robot = robotHardware;
		} else if (leftJoystick.getRawButtonPressed(7)){
			autoProgram = new MASideAuto();
			autoProgram.LeftSide = 1;
            autoProgram.lastStep = 7;
			autoProgram.robot = robotHardware;
		} else if (leftJoystick.getRawButtonPressed(8)){
			autoProgram = new MASideAuto();
			autoProgram.LeftSide = -1;
            autoProgram.lastStep = 7;
			autoProgram.robot = robotHardware;
		} else if (leftJoystick.getRawButtonPressed(9)) {
		    autoProgram = new DriveStraightAuto();
            autoProgram.robot = robotHardware;
            autoProgram.lastStep = 1;
        } else if (leftJoystick.getRawButtonPressed(10)) {
		    autoProgram = new MASideAutoSimpleArm();
		    autoProgram.robot = robotHardware;
		    autoProgram.lastStep = 10;
        }
		/*else if (leftJoystick.getRawButton(9)){
			autoProgram = new MOErioCargoSideAutoBonus();
			autoProgram.LeftSide = 1;
			autoProgram.robot = robotHardware;
		} else if (leftJoystick.getRawButton(10)){
			autoProgram = new MOErioCargoSideAutoBonus();
			autoProgram.LeftSide = -1;
			autoProgram.robot = robotHardware;
		}*/
		if (functionStick.getStickButtonPressed(Hand.kLeft)) {
			functionStickDrive = !functionStickDrive;
		}

	}

	@Override
	public void autonomousInit () {
		robotHardware.shiftLow();
        autoEnable = true; //change if auto is teleop
		autoProgram.init();
	}

	@Override
	public void autonomousPeriodic () {
	    if (autoEnable) {
	        autoProgram.run();
            //startAutoStep = autoProgram.autoStep;
        } else teleopPeriodic();
        if ((leftJoystick.getRawButtonPressed(8)) || (autoProgram.autoStep == autoProgram.lastStep)) {
            autoEnable = false;
            teleopInit();
        }
	}

	@Override
	public void teleopInit () {
		robotHardware.shiftLow();
        //autoProgram.autoStep = startAutoStep;
        ClimbEncoderOrigin = robotHardware.getClimberLEncoderCount();
		autoEnable = false;
		climbEnabled = false;
        atHabHeight3 = false;
        atHabHeight2 = false;
        climbAuto.init();
	}

	@Override
	public void teleopPeriodic () {
	    //initial auto

		if (autoEnable) {
            autoProgram.run();
            if (leftJoystick.getRawButtonPressed(8))
                autoEnable = false;
        } else if (climbEnabled) {
			climbAuto.run();
			if (leftJoystick.getRawButtonPressed(8))
				climbEnabled = false;
		} else {
			//Driving Adjustments
			if (functionStick.getStickButtonPressed(Hand.kLeft)) {
				functionStickDrive = !functionStickDrive;
			}

			if (leftJoystick.getTrigger())  robotHardware.moveForward     (.25);
			else if (leftJoystick.getRawButton(3))  robotHardware.turnLeftInplace (.25);
			else if (leftJoystick.getRawButton(2)) {
				switch (step) {
					case 1:
						currentEncoder = robotHardware.getDistanceLeftInches();
						step++;
						break;
					case 2:
						if (robotHardware.getDistanceLeftInches() < currentEncoder - 0.25) {
							robotHardware.moveBackward(0);
							step = 1;
						}
						else robotHardware.moveBackward(.25);
						break;
				}
			}
			else if (leftJoystick.getRawButton(4))  robotHardware.turnRightInplace(.25);

			//Manual Control
			else {
				if (!functionStickDrive) {
					driveJoyStickX = leftJoystick.getX();
					driveJoyStickY = -leftJoystick.getY();
				} else {
					driveJoyStickX =  functionStick.getX(Hand.kLeft);
					driveJoyStickY = -functionStick.getY(Hand.kLeft);
				}

				/*if (Math.abs(driveJoyStickY) < 0.05) driveJoyStickY = 0.0;
				else if (driveJoyStickX > 0) driveJoyStickX -= 0.05;
				else if (driveJoyStickX < 0) driveJoyStickX += 0.05;
				//Attempt to drive straight if joystick is within 15% of vertical
				if (Math.abs(driveJoyStickX) < 0.15) driveJoyStickX = 0.0;
				else if (driveJoyStickX > 0) driveJoyStickX -= 0.15;
				else if (driveJoyStickX < 0) driveJoyStickX += 0.15;*/

				if (Math.abs(driveJoyStickY) < 0.03) driveJoyStickY = 0.0;
				else if (driveJoyStickX > 0) driveJoyStickX -= 0.03;
				else if (driveJoyStickX < 0) driveJoyStickX += 0.03;
				//Attempt to drive straight if joystick is within 10% of vertical
				if (Math.abs(driveJoyStickX) < 0.10) driveJoyStickX = 0.0;
				else if (driveJoyStickX > 0) driveJoyStickX -= 0.1;
				else if (driveJoyStickX < 0) driveJoyStickX += 0.1;

				//driveJoyStickY *= 1.052631579;
				driveJoyStickY *= 1.15;
				driveJoyStickX *= .95;

				double drivePowerLeft  = driveJoyStickY + driveJoyStickX;
				double drivePowerRight = driveJoyStickY - driveJoyStickX;

				robotHardware.setDrivePower(drivePowerLeft, drivePowerRight);
			}

			if (functionStick.getStartButton()) {
				switch (step) {
					case 1:
						currentEncoder = robotHardware.getDistanceLeftInches();
						step++;
						break;
					case 2:
						if (robotHardware.getDistanceLeftInches() < currentEncoder - 0.25) {
							robotHardware.moveBackward(0);
							step = 1;
						}
						else robotHardware.moveBackward(.25);
						break;
				}
			}

			//Shifting
			if (leftJoystick.getRawButtonPressed (12)) robotHardware.shiftHigh();
			if (leftJoystick.getRawButtonReleased(12)) robotHardware.shiftLow ();

			//hatchGrab
			if      (functionStick.getAButton()) robotHardware.spearIn    ();
			else if (functionStick.getBButton()) robotHardware.spearOut   ();
			if      (functionStick.getXButton()) robotHardware.spearHook  ();
			else if (functionStick.getYButton()) robotHardware.spearUnhook();

			//roller
			//TODO: bumpers
			if      (functionStick.getBumper(Hand.kLeft )) robotHardware.rollOut (0.5);
			else if (functionStick.getBumper(Hand.kRight)) robotHardware.rollIn(0.8);
			else                                           robotHardware.driveRoller(0.0);

			//arm
			double armPower = functionStick.getY(Hand.kRight);
			if (Math.abs(armPower) < 0.2) armPower = 0;
			else if (armPower > 0) armPower -= 0.2;
			else if (armPower < 0) armPower += 0.2;
			if (armPower != 0) noPosition();
			robotHardware.driveArm(-armPower);


			//elevator
			//elevator position = -29.7
			double elevatorPower =
					functionStick.getTriggerAxis(Hand.kRight) - functionStick.getTriggerAxis(Hand.kLeft );

			if (Math.abs(elevatorPower) < 0.3) elevatorPower = 0;
			else if (elevatorPower > 0) elevatorPower -= 0.3;
			else if (elevatorPower < 0) elevatorPower += 0.3;
			robotHardware.driveElevator(elevatorPower*0.8);


			//Climbing
			if (leftJoystick.getRawButtonPressed(8)) {
				climbEnabled = true;
			}
			if (leftJoystick.getRawButton(6)) {
				robotHardware.climb(-1.0);
			} else if (leftJoystick.getRawButton(9)) {
				robotHardware.climb(0.4);
			} else if (leftJoystick.getRawButton(15)) {
				robotHardware.climbRDown(0.3);
			}
			else if (leftJoystick.getRawButton(16)) {
				robotHardware.climbLDown(0.3);
			} else {
				//TEST these first two lines
				if (functionStick.getX(Hand.kRight) > 0.9) robotHardware.climb(-1.0);
				else if (functionStick.getX(Hand.kRight) < -0.9) robotHardware.climb(-0.3);
				else robotHardware.climb(0);
			}

			//Foot Toggle
			if (leftJoystick.getRawButtonPressed(11)) {
				footToggle = !footToggle;
			}
			if (functionStick.getStickButtonPressed(Hand.kRight)) {
				footToggle = !footToggle;
			}
			if (footToggle) robotHardware.LinearSlider(DoubleSolenoid.Value.kForward);
			else robotHardware.LinearSlider(DoubleSolenoid.Value.kReverse);

			//Auto Positioning
			if (switchBox.getRawButtonPressed(5)) cargo = true;
			else if (switchBox.getRawButtonReleased(5)) cargo = false;

			if (switchBox.getRawButtonPressed(2)) {
				positionLock = true;
				pos = 0;
			}
			else if (switchBox.getRawButtonPressed(3)) {
				positionLock = true;
				pos = 1;
			}
			else if (switchBox.getRawButtonPressed(4)) {
				positionLock = true;
				pos = 2;
			}

			if (positionLock) {
				if (cargo) cargoPos[pos].run();
				else hatchPos[pos].run();
			}

			//DPAD
			/*POVDirection controlPadDirection = POVDirection.getDirection(functionStick.getPOV());
			switch (controlPadDirection) {
				case NORTH:
					cargoPos[0] = false;
					cargoPos[1] = false;
					cargoPos[2] = false;
					hatchPos[0] = false;
					hatchPos[1] = true;
					hatchPos[2] = false;
					break;
				case SOUTH:
					cargoPos[0] = false;
					cargoPos[1] = false;
					cargoPos[2] = false;
					hatchPos[0] = true;
					hatchPos[1] = false;
					hatchPos[2] = false;
					break;
				case EAST:
					cargoPos[0] = false;
					cargoPos[1] = true;
					cargoPos[2] = false;
					hatchPos[0] = false;
					hatchPos[1] = false;
					hatchPos[2] = false;
					break;
				case WEST:
					cargoPos[0] = true;
					cargoPos[1] = false;
					cargoPos[2] = false;
					hatchPos[0] = false;
					hatchPos[1] = false;
					hatchPos[2] = false;
					break;
				case NORTHWEST:
				case SOUTHEAST:
				case NORTHEAST:
				case SOUTHWEST:
				default:
					break;
			}

			if (hatchPos[0]) {
				hatch1.run();
			} else if (hatchPos[1]) {
				hatch2.run();
			} else if (cargoPos[0]) {
				cargo1.run();
			} else if (cargoPos[1]) {
				cargo2.run();
			}*/


			POVDirection joystickPOV = POVDirection.getDirection(leftJoystick.getPOV());
			switch (joystickPOV) {
				case NORTH:
				case SOUTH:
				case EAST:
				case WEST:
				case NORTHWEST:
				case SOUTHEAST:
				case NORTHEAST:
				case SOUTHWEST:
				default:
					break;
			}
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
