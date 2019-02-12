package frc.robot.genericrobot;

/*code*/

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public abstract class GenericRobot {
    //movement of robot

    double leftPower;
    double rightPower;
    double elevatorPower;
    double turretPower;
    double armPower;

    public void moveForward(double motorPower) {
        setDrivePower(motorPower, motorPower);
    }
    public void moveBackward(double motorPower) {
        setDrivePower(-motorPower, -motorPower);
    }
    public void turnLeftInplace(double motorPower) {
        setDrivePower(-motorPower, motorPower);
    }
    public void turnRightInplace(double motorPower) {
        setDrivePower(motorPower, -motorPower);
    }
    abstract void setDrivePowerInternal(double leftMotor, double rightMotor);
    abstract void setElevatorInternal(double power);
    abstract void setTurretInternal(double power);
    abstract void setArmInternal(double power);
    public final void setDrivePower(double leftMotor, double rightMotor) {
        leftPower = leftMotor;
        rightPower = rightMotor;
        setDrivePowerInternal(leftMotor, rightMotor);
    }

   //checking for things
    public abstract double getDistanceLeftInches();
    public abstract double getDistanceRightInches();
    public abstract double getHeadingDegrees();

    public double getLeftDrivePower () { return leftPower; }
    public double getRightDrivePower() { return rightPower; }

    //stopping and resetting
    public abstract void resetDriveEncoders();
    public abstract void resetYaw();
    public abstract void stopEverything();
    public abstract void stopDriving();

    //moving the elevator
    public void elevatorUp(double power) {driveElevator(power);}
    public void elevatorDown(double power) {driveElevator(-power);}
    public void turretLeft(double power) {driveTurret(power);}
    public void turretRight(double power){driveTurret(-power);}

    public final void driveElevator(double power) {
        elevatorPower = power;
        if (isElevatorUp() && power > 0) {
            setElevatorInternal(0);
        } else if (isElevatorUp() && power < 0) {
            setElevatorInternal(0);
        } else {
            setElevatorInternal(power);
        }
    }

    public final void driveTurret(double power) {
        turretPower = power;
        if (isTurretRight() && power > 0) {
            setTurretInternal(0);
        } else if (isTurretLeft() && power < 0) {
            setTurretInternal(0);
        } else {
            setTurretInternal(power);
        }
    }

    public final void driveArm(double power) {
        armPower = power;
        if (isArmUp() && power > 0) {
            setArmInternal(0);
        } else if (isArmDown() && power < 0) {
            setArmInternal(0);
        } else {
            setArmInternal(power);
        }
    }

    public boolean isElevatorUp() {return false;}
    public boolean isElevatorDown() {return false;}

    public double getElevatorPower() {return elevatorPower;}

    public boolean isTurretRight() {return false;}
    public boolean isTurretLeft() {return false;}

    public double getTurretPower() {return turretPower;}

    public boolean isArmUp() {return false;}
    public boolean isArmDown() {return false;}

    public double getArmPower() {return armPower;}

    public abstract double getElevatorEncoderCount();
    public abstract double getArmEncoderCount();
    public abstract double getTurretEncoderCount();

    //grabbers
    public abstract void driveRoll(double power);
    public abstract void rollIn();
    public abstract void rollOut();

    public abstract void grabHatch();

    public abstract void checkSafety();

    //just for now
    public abstract void driveSA(double power);
    public abstract void driveSB(double power);
    public abstract void driveFA(double power);
    public abstract void driveFB(double power);

    //smartdashboard?
    private static Timer smartDashboardTimer = new Timer() {{
        start();
    }};

    public static void dashboardInit(GenericRobot robotHardware) {
        setDefaultValues(robotHardware);
        printToSmartDashboard(robotHardware);
        //getFromSmartDashboard(robotHardware);
    }

    public static void dashboardPeriodic(GenericRobot robotHardware) {
        if (smartDashboardTimer.hasPeriodPassed(0.3)) {
            printToSmartDashboard(robotHardware);
            //getFromSmartDashboard(robotHardware);
            smartDashboardTimer.reset();
            //smartDashboardTimer.start();
        }
    }

    static void setDefaultValues(GenericRobot robotHardware) {
        SmartDashboard.putNumber("Yaw: ", robotHardware.getHeadingDegrees());
        SmartDashboard.putNumber("Left Encoder: ", robotHardware.getDistanceLeftInches());
        SmartDashboard.putNumber("Right Encoder: ", robotHardware.getDistanceRightInches());
        SmartDashboard.putNumber("Left Drive Power: ", robotHardware.getLeftDrivePower());
        SmartDashboard.putNumber("Right Drive Power: ", robotHardware.getRightDrivePower());
        //SmartDashboard.putNumber("autostep: ", robotHardware.autoStep);
    }

    static void getFromSmartDashboard(GenericRobot robotHardware) {

    }

    static void printToSmartDashboard(GenericRobot robotHardware) {

    }
}

