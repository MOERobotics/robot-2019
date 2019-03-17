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
import edu.wpi.first.cameraserver.CameraServer;

import java.util.Arrays;

import java.util.Date;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;

public class Robot extends TimedRobot {

	private GenericRobot   robotHardware = new SuperMOEva();
	private Joystick       leftJoystick  = new Joystick(0);
	private XboxController functionStick = new XboxController(1);
	private GenericAuto    autoProgram   = new DeployArm();

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
	static final double upperElevator = 1;
	static final double bottomElevator = -0.6;

	/* kP = 0.1, kI = 8*10^-3, kD = 0.0*/

	public double ClimbEncoderOrigin = 0;
    static final double HABheight2 = 20;
	static final double HABheight3 = 20;
	boolean atHabHeight2 = false;
	boolean atHabHeight3 = false;
	private boolean footToggle = false;
	private boolean climbPushToggle = false;

	private boolean functionStickDrive = false;
    double driveJoyStickX;
    double driveJoyStickY;

	int step = 1;
	double currentEncoder;

	@Override
	public void robotInit() {
	    //robotHardware.shiftDriveInternal(true);
		autoProgram.robot = robotHardware;
		autoProgram.LeftSide = 1;
        robotHardware.climbPushForwardz(DoubleSolenoid.Value.kOff);
        robotHardware.climb2(DoubleSolenoid.Value.kOff);
		//robotHardware.shiftLow();
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

		if (PortOpen) Lidar.init(Blinky);
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

            SmartDashboard.putBoolean("Is ArmDown: ", robotHardware.isArmDown());
            SmartDashboard.putBoolean("Is ArmUp: ", robotHardware.isArmUp());
            SmartDashboard.putBoolean("Is Elevator Down: ", robotHardware.isElevatorDown());
            SmartDashboard.putBoolean("Is Elevator Up: ", robotHardware.isElevatorUp());
            SmartDashboard.putBoolean("SAFETY MOEVERRIDE", robotHardware.getSafetyOverride());

            SmartDashboard.putBoolean("Shifter State: ", robotHardware.getShifterSolenoidState());
            SmartDashboard.putBoolean("Spear State: ", robotHardware.getSpearShaftState());
            SmartDashboard.putBoolean("Hatch Grabber State: ", robotHardware.getSpearHookState());
            SmartDashboard.putBoolean("Floor Pickup State: ", robotHardware.getFloorPickupState());

            SmartDashboard.putBoolean("Elevator Forward Limit Enabled: ", robotHardware.isElevForwardLimitEnabled());
            SmartDashboard.putBoolean("Elevator Reverse Limit Enabled: ", robotHardware.isElevReverseLimitEnabled());
            SmartDashboard.putBoolean("Arm Forward Limit Enabled: ", robotHardware.isArmForwardLimitEnabled());
            SmartDashboard.putBoolean("Arm Reverse Limit Enabled: ", robotHardware.isArmReverseLimitEnabled());

            SmartDashboard.putNumber("Climber Power: ", robotHardware.getClimbPower());
            SmartDashboard.putNumber("Climber Left Encoder: ", robotHardware.getClimberLEncoderCount());
            SmartDashboard.putNumber("Climber Right Encoder: ", robotHardware.getClimberREncoderCount());
			SmartDashboard.putBoolean("Foot Toggle: ", footToggle);
            SmartDashboard.putString("Climb Push Forwardz State: ", robotHardware.getClimbPushForwardzState().name());
            SmartDashboard.putBoolean("Climb Push Toggle: ", climbPushToggle);
            SmartDashboard.putString("Feet Out: ", robotHardware.getClimb2State().name());

            SmartDashboard.putBoolean("DIO 6: ", robotHardware.getSix());
            SmartDashboard.putBoolean("DIO 7: ", robotHardware.getSeven());

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

		/*if (leftJoystick.getRawButtonPressed (12)) robotHardware.setSafetyOverride(true);
		if (leftJoystick.getRawButtonReleased(12)) robotHardware.setSafetyOverride(false);*/
		//if leftJoystick.getRawButtonPressed(12) {robotHardware.climbPushForwardz(DoubleSolenoid.Value.kForward);}
		//if leftJoystick.getRawButtonPressed(12) {robotHardware.climbPushForwardz(DoubleSolenoid.Value.kReverse);}

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
		} else if (leftJoystick.getRawButtonPressed(14)){
			autoProgram = new MAFrontAuto();
			autoProgram.LeftSide = -1;
			autoProgram.lastStep = 4;
			autoProgram.robot = robotHardware;
		} else if (leftJoystick.getRawButtonPressed(12)){
			autoProgram = new MASideAuto();
			autoProgram.LeftSide = 1;
            autoProgram.lastStep = 7;
			autoProgram.robot = robotHardware;
		} else if (leftJoystick.getRawButtonPressed(15)){
			autoProgram = new MASideAuto();
			autoProgram.LeftSide = -1;
            autoProgram.lastStep = 7;
			autoProgram.robot = robotHardware;
		} else if (leftJoystick.getRawButtonPressed(14)) {
		    autoProgram = new DriveStraightAuto();
            autoProgram.robot = robotHardware;
            autoProgram.lastStep = 1;
        } else if (leftJoystick.getRawButtonPressed(16)) {
		    autoProgram = new UnitTestArcZ();
		    autoProgram.robot = robotHardware;
		    autoProgram.lastStep = 1;
        }
		/*else if (leftJoystick.getRawButtonPressed(9)){
			autoProgram = new MOErioCargoSideAutoBonus();
			autoProgram.LeftSide = 1;
			autoProgram.robot = robotHardware;
		} else if (leftJoystick.getRawButtonPressed(10)){
			autoProgram = new MOErioCargoSideAutoBonus();
			autoProgram.LeftSide = -1;
			autoProgram.robot = robotHardware;
		}*/

		if (functionStick.getStickButton(Hand.kLeft)) {
			functionStickDrive = !functionStickDrive;
		}

	}

	@Override
	public void autonomousInit () {
        autoEnable = true; //change if auto is teleop
		autoProgram.init();
	}

	@Override
	public void autonomousPeriodic () {
	    if (autoEnable) {
	        autoProgram.run();
            //startAutoStep = autoProgram.autoStep;
        } else teleopPeriodic();
        if ((leftJoystick.getRawButton(6)) || (autoProgram.autoStep == autoProgram.lastStep)) {
            autoEnable = false;
            teleopInit();
        }
	}

	@Override
	public void teleopInit () {
        //autoProgram.autoStep = startAutoStep;
        atHabHeight3 = false;
        atHabHeight2 = false;
	}

	@Override
	public void teleopPeriodic () {
	    //initial auto
        autoEnable = false;

        /*if (autoEnable) {
            autoProgram.run();
            if (leftJoystick.getRawButton(8))
                autoEnable = false;
        }*/

	    //Driving Adjustments
		if (leftJoystick.getTrigger()   )  robotHardware.moveForward     (.25);
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
                    else robotHardware.moveBackward    (.25);
                    break;
            }

        }
		else if (leftJoystick.getRawButton(4))  robotHardware.turnRightInplace(.25);

		//Individual motors (For testing) - SET TO COAST IF USED
		//else if (leftJoystick.getRawButton(5))  robotHardware.driveSA(0.5);
		//else if (leftJoystick.getRawButton(6))  robotHardware.driveSB(0.5);
		//else if (leftJoystick.getRawButton(7))  robotHardware.driveFA(0.5);
		//else if (leftJoystick.getRawButton(8))  robotHardware.driveFB(0.5);

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
            driveJoyStickY *= 1.10;
            driveJoyStickX *= .85;

            double drivePowerLeft  = driveJoyStickY + driveJoyStickX;
            double drivePowerRight = driveJoyStickY - driveJoyStickX;

            robotHardware.setDrivePower(drivePowerLeft, drivePowerRight);
		}

		if (functionStick.getStickButtonPressed(Hand.kLeft)) {
		    functionStickDrive = !functionStickDrive;
        }

        if (functionStick.getStickButtonPressed(Hand.kRight)) {
            footToggle = !footToggle;
            if (footToggle) robotHardware.climb2(DoubleSolenoid.Value.kForward);
            else robotHardware.climb2(DoubleSolenoid.Value.kReverse);
        }

        if (leftJoystick.getRawButtonPressed(7)) {
            climbPushToggle = !climbPushToggle;
            if (climbPushToggle) robotHardware.climbPushForwardz(DoubleSolenoid.Value.kForward);
            else robotHardware.climbPushForwardz(DoubleSolenoid.Value.kReverse);
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

		//Climbing
		if (leftJoystick.getRawButton(6)) {
			robotHardware.climb(-1.0);
		} else if (leftJoystick.getRawButton(9)) {
			robotHardware.climb(0.4);
		}
		else if (leftJoystick.getRawButton(15))
		{
			robotHardware.climbRDown(0.3);
		}
		else if (leftJoystick.getRawButton(16))
		{
			robotHardware.climbLDown(0.3);
		}
		else
		{
			robotHardware.climb(0);

		}
		/*else {
            if (functionStick.getX(Hand.kRight) > 0.9) robotHardware.climb(-1.0);
            else if (functionStick.getX(Hand.kRight) < -0.9) robotHardware.climb(-0.3);
			else robotHardware.climb(0);
		}*/

		/*
		if (Math.abs(robotHardware.getClimberLEncoderCount()-ClimbEncoderOrigin) < HABheight) {
            if      (leftJoystick.getRawButton( 9)) robotHardware.climb  (0.5);
            else if (leftJoystick.getRawButton(10)) robotHardware.climb(-1.0);


                //else if (controlPadDirection == POVDirection.EAST)   robotHardware.climbRDown(0.3);
                //else if (controlPadDirection == POVDirection.WEST)   robotHardware.climbLDown(0.3);

            else                                    robotHardware.climb    (0.0);
        }
        if (Math.abs(robotHardware.getClimberLEncoderCount()-ClimbEncoderOrigin) >= HABheight) {
            if (leftJoystick.getRawButton(10))
            {
                robotHardware.climb2(true);
            }
        }
	    */

		//Shifting
        /*if (leftJoystick.getRawButtonPressed(11)) {
            footToggle = !footToggle;
            if (footToggle) robotHardware.climb2(DoubleSolenoid.Value.kForward);
            else robotHardware.climb2(DoubleSolenoid.Value.kReverse);
        }*/

        if (leftJoystick.getRawButtonPressed (8)) robotHardware.shiftHigh();
		if (leftJoystick.getRawButtonReleased(8)) robotHardware.shiftLow ();

		//hatchGrab
		if      (functionStick.getAButton()) robotHardware.spearIn    ();
		else if (functionStick.getBButton()) robotHardware.spearOut   ();
		if      (functionStick.getXButton()) robotHardware.spearHook  ();
		else if (functionStick.getYButton()) robotHardware.spearUnhook();

		//roller
		if      (functionStick.getBumper(Hand.kLeft )) robotHardware.rollOut (0.5);
		else if (functionStick.getBumper(Hand.kRight)) robotHardware.rollIn(0.8);
		else                                           robotHardware.driveRoller(0.0);

		//arm
		//Right stick, up/down rotates.
		double armPower  = functionStick.getY(Hand.kRight);
		if (Math.abs(functionStick.getX(Hand.kRight)) > 0.9) armPower = 0;

		if (
			Math.abs(armPower) < 0.2
		) armPower = 0;

		else if (armPower > 0) armPower -= 0.2;
		else if (armPower < 0) armPower += 0.2;
		robotHardware.driveArm(-armPower);

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

/*
		POVDirection controlPadDirection = POVDirection.getDirection(functionStick.getPOV());
		SmartDashboard.putString("DPAD", controlPadDirection.name());
		SmartDashboard.putNumber("DPAD Direction", functionStick.getPOV());
		switch (controlPadDirection) {
			case NORTH:
				robotHardware.climb(0.3);
				break;
			case SOUTH:
				robotHardware.climb(1.0);
				break;
			case EAST:
				robotHardware.climbRDown(0.3);
				break;
			case WEST:
				robotHardware.climbLDown(0.3);
				break;
			case NORTHWEST:
			case SOUTHEAST:
			case NORTHEAST:
			case SOUTHWEST:
			default:
				break;
		}
		*/

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
