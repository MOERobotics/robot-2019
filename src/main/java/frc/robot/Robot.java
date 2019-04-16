/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.*;
import frc.robot.autonomous.climb.AutoFloatingFullRetraction;
import frc.robot.autonomous.climb.AutoFlyingFullRetraction;
import frc.robot.autonomous.presets.*;
import frc.robot.autonomous.test.*;
import frc.robot.autonomous.sandstorm.*;
import frc.robot.autonomous.visionAutos.PivotApproach;
import frc.robot.autonomous.visionAutos.PivotBot;
import frc.robot.genericrobot.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.vision.PiClient;

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

    private PiClient piClient = PiClient.getInstance();

	private GenericAuto 	autoProgram   = new MARocketHatch1Auto();
	private GenericAuto	    hab3Climb 	 = new AutoFlyingFullRetraction();
	private GenericAuto     hab2Climb     = new AutoFloatingFullRetraction();
	private GenericAuto 	pixyAlign 	= new PivotBot();
	private GenericAuto 	pixyApproach = new PivotApproach();

	//presets
	private GenericAuto[] cargoPos = {new Cargo1(), new Cargo2(), new Cargo3()};
	private GenericAuto[] hatchPos = {new Hatch1(), new Hatch2(), new Hatch3()};
	int pos = -1;
	boolean cargo = false;
	boolean positionLock = false;

	//UsbCamera cam1;
    int smartDashCounter = 0;

	//lidar
	SerialPort Blinky;
	boolean PortOpen = false;

	//auto
    boolean autoEnable = true;
    //int startAutoStep;

	public double ClimbEncoderOrigin = 0;
	boolean atHabHeight2 = false;
	boolean atHabHeight3 = false;
	private boolean footToggle = true;
	private boolean hab2Enabled = false;
	private boolean hab3Enabled = false;

	private boolean functionStickDrive = false;
    double driveJoyStickX;
    double driveJoyStickY;
    private boolean shiftingHigh;

    private boolean pixyAligning = false;
    private boolean pixyApproaching = false;

    //for the lil nudge
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
		autoProgram.robot = robotHardware;
		pixyAlign.init();
		pixyAlign.robot = robotHardware;
		pixyApproach.init();
		pixyApproach.robot = robotHardware;

		hab2Climb.robot = robotHardware;
		hab3Climb.robot = robotHardware;

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

		if (PortOpen) LidarWithThread.init(Blinky, robotHardware);
	}

	@Override
	public void robotPeriodic () {
        //robotHardware.checkSafety();

	    if (0==(smartDashCounter++ % 10)) { //-Brian
	        //robotHardware.getClimberCurrent();
            SmartDashboard.putString("Robot Class", robotHardware.getClass().getSimpleName());
            SmartDashboard.putString("Auto Class", autoProgram.getClass().getSimpleName());

            SmartDashboard.putNumber("Yaw: ", robotHardware.getHeadingDegrees());
            SmartDashboard.putNumber("Roll: ", robotHardware.getRollDegrees());
            SmartDashboard.putNumber("Pitch: ", robotHardware.getPitchDegrees());
            //SmartDashboard.putNumber("Left Encoder (Raw): ", robotHardware.getDistanceLeftInches() * 462);
            //SmartDashboard.putNumber("Right Encoder (Raw): ", robotHardware.getDistanceRightInches() * 462);
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

            //SmartDashboard.putBoolean("ALEX DRIVE", functionStickDrive);

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
            SmartDashboard.putBoolean("Auto Hab 2 Climb Enabled: ", hab2Enabled);
            SmartDashboard.putBoolean("Auto Hab 3 Climb Enabled", hab3Enabled);
            SmartDashboard.putString("Auto Hab 2 Climb: ", hab2Climb.getClass().getSimpleName());
            SmartDashboard.putString("Auto Hab 3 Climb: ", hab3Climb.getClass().getSimpleName());

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
            //autoProgram.printSmartDashboard();

            SmartDashboard.putString("PixyInfo: ", robotHardware.pixy.toString());
        }

		if (leftJoystick.getRawButtonPressed (11)) robotHardware.setOffsets();
		if (leftJoystick.getRawButtonReleased(11)) robotHardware.clearOffsets();
		if (leftJoystick.getThrottle() < -0.95) robotHardware.setSafetyOverride(true);
		else if (leftJoystick.getThrottle() > 0.95) robotHardware.setSafetyOverride(false);

		/*robotHardware.piXY = piClient.getCentroidXY();

        SmartDashboard.putNumber("Vision_X:" , robotHardware.piXY[0]);
        SmartDashboard.putNumber("Vision_Y:" , robotHardware.piXY[1]);*/

		//if (PortOpen) Lidar.getLidar(robotHardware, Blinky);
		if (PortOpen) LidarWithThread.getLidar(robotHardware);
	}

	@Override
	public void disabledInit () {
		// This is to prevent the shaft from pulling back during the Sandstorm transition to teleop.
		// robotHardware.shiftSpearHook(false);
		robotHardware.shiftSpearShaft(false);
	}

	@Override
	public void disabledPeriodic () {
		ClimbEncoderOrigin = robotHardware.getClimberLEncoderCount();

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
			autoProgram.lastStep = 4234;
			autoProgram.robot = robotHardware;
		} else if (leftJoystick.getRawButtonPressed(6)){
			autoProgram = new MAShipFrontHatch1Auto();
			autoProgram.LeftSide = -1;
			autoProgram.lastStep = 4342;
			autoProgram.robot = robotHardware;
		} else if (leftJoystick.getRawButtonPressed(7)){
			autoProgram = new MASideAutoPixy();
			autoProgram.LeftSide = 1;
            autoProgram.lastStep = 429;
			autoProgram.robot = robotHardware;
		} else if (leftJoystick.getRawButtonPressed(8)){
			autoProgram = new MASideAutoPixy();
			autoProgram.LeftSide = -1;
            autoProgram.lastStep = 659;
			autoProgram.robot = robotHardware;
		} else if (leftJoystick.getRawButtonPressed(9)) {
		    autoProgram = new DriveStraightAuto();
            autoProgram.robot = robotHardware;
            autoProgram.lastStep = 4329;
        } else if (leftJoystick.getRawButtonPressed(10)) {
		    autoProgram = new MASideAutoSimpleArm();
		    autoProgram.robot = robotHardware;
		    autoProgram.lastStep = 1232;
        } else if (leftJoystick.getRawButtonPressed(12)) {
		    autoProgram = new DeployArm();
		    autoProgram.robot = robotHardware;
		    autoProgram.lastStep = 2342;
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
		/*if (functionStick.getStickButtonPressed(Hand.kLeft)) {
			functionStickDrive = !functionStickDrive;
		}*/

	}

	@Override
	public void autonomousInit () {
		robotHardware.shiftHigh();
        autoEnable = true;
		autoProgram.init();
	}

	@Override
	public void autonomousPeriodic () {
		if (autoEnable) {
	        autoProgram.run();
            //startAutoStep = autoProgram.autoStep;
        } else teleopPeriodic();
        if ((leftJoystick.getRawButtonPressed(5)) || (autoProgram.autoStep == autoProgram.lastStep)) {
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
		hab2Enabled = false;
		hab3Enabled = false;

        atHabHeight3 = false;
        atHabHeight2 = false;
        hab2Climb.init();
        hab3Climb.init();

        pixyAlign.init();
        autoProgram.init();

        shiftingHigh = true;
        robotHardware.shiftLow();
	}

	@Override
	public void teleopPeriodic () {
	    //Initial auto
		if (autoEnable) {
            autoProgram.run();
            if (leftJoystick.getRawButtonPressed(5))
                autoEnable = false;
        }
		//Climbing - all drive functionalities disabled
        else if (hab2Enabled) {
			hab2Climb.run();
			if (leftJoystick.getRawButtonPressed(14))
				hab2Enabled = false;
		} else if (hab3Enabled) {
		    hab3Climb.run();
		    if (leftJoystick.getRawButtonPressed(8))
		        hab3Enabled = false;
        }
        //Normal driving, functions
        else {
			//Driving Adjustments
			/*if (functionStick.getStickButtonPressed(Hand.kLeft)) {
				functionStickDrive = !functionStickDrive;
			}*/

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

				if (pixyAligning) {
					pixyAlign.run();
					if ((Math.abs(driveJoyStickY) > 0.03)|| Math.abs(driveJoyStickX) > 0.10) {
						pixyAligning = false;
					}
				} else if (pixyApproaching) {
					pixyApproach.run();
					if ((Math.abs(driveJoyStickY) > 0.03)|| Math.abs(driveJoyStickX) > 0.10) {
						pixyApproaching = false;
					}
				} else {
					if (Math.abs(driveJoyStickY) < 0.03) driveJoyStickY = 0.0;
					else if (driveJoyStickX > 0) driveJoyStickX -= 0.03;
					else if (driveJoyStickX < 0) driveJoyStickX += 0.03;
					//Attempt to drive straight if joystick is within 10% of vertical
					if (Math.abs(driveJoyStickX) < 0.10) driveJoyStickX = 0.0;
					else if (driveJoyStickX > 0) driveJoyStickX -= 0.1;
					else if (driveJoyStickX < 0) driveJoyStickX += 0.1;

					driveJoyStickY *= 1.15;
					driveJoyStickX *= .95;

					double drivePowerLeft  = driveJoyStickY + driveJoyStickX;
					double drivePowerRight = driveJoyStickY - driveJoyStickX;

					robotHardware.setDrivePower(drivePowerLeft, drivePowerRight);
				}
			}

			if (functionStick.getStartButton()) {
				robotHardware.resetElevatorPosition();
			}

			//Shifting
			if (leftJoystick.getRawButtonPressed(5)) shiftingHigh = true;
			else if (leftJoystick.getRawButtonPressed(10)) shiftingHigh = false;

			if (shiftingHigh) robotHardware.shiftLow(); //this makes no sense now – need to switch mechanically
			else robotHardware.shiftHigh();

			//hatchGrab
			if      (functionStick.getAButton()) robotHardware.spearHook ();
			else if (functionStick.getBButton()) robotHardware.spearUnhook();
			if      (functionStick.getXButton()) robotHardware.spearIn    ();
			else if (functionStick.getYButton()) robotHardware.spearOut   ();

			//roller
			//TODO: bumpers
			if      (functionStick.getBumper(Hand.kLeft )) robotHardware.rollOut (0.5);
			else if (functionStick.getBumper(Hand.kRight)) robotHardware.rollIn(0.8);
            else                                           robotHardware.rollIn(0.1);

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
			if (elevatorPower != 0) noPosition();
			robotHardware.driveElevator(elevatorPower*0.8);

			//Climbing
			if (leftJoystick.getRawButtonPressed(8)) {
				hab3Enabled = true;
			} else if (leftJoystick.getRawButtonPressed(14)) {
			    hab2Enabled = true;
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
				//if (functionStick.getX(Hand.kRight) > 0.9) robotHardware.climb(-1.0);
				//else if (functionStick.getX(Hand.kRight) < -0.9) robotHardware.climb(-0.3);
				//else
				robotHardware.climb(0);
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
            if (!positionLock) {
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
            } else if (positionLock && pos != -1) {
                if (cargo) cargoPos[pos].run();
                else hatchPos[pos].run();
            }


            //DPAD
			POVDirection controlPadDirection = POVDirection.getDirection(functionStick.getPOV());
			switch (controlPadDirection) {
				case NORTH:
					break;
				case SOUTH:
					break;
				case EAST:
					break;
				case WEST:
					break;
				case NORTHWEST:
				case SOUTHEAST:
				case NORTHEAST:
				case SOUTHWEST:
				default:
					break;
			}

			POVDirection joystickPOV = POVDirection.getDirection(leftJoystick.getPOV());
			switch (joystickPOV) {
				case NORTH:
				    if (robotHardware.pixy.vec.length != 0) {
                        pixyAligning = true;
                        pixyAlign.init();
                    }
					break;
				case SOUTH:
				    if (robotHardware.pixy.vec.length != 0) {
                        pixyApproaching = true;
                        pixyApproach.init();
                    }
					break;
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
