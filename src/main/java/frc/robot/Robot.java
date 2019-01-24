/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {

    CaMOElot robotHardware = new CaMOElot();
    Joystick leftJoystick = new Joystick(0);
    public boolean buttonTwoIsPressed = leftJoystick.getRawButtonPressed(2);
    public boolean buttonOneIsPressed = leftJoystick.getRawButtonPressed(1);

    public StraightAccel thingy = new StraightAccel();


    @Override
    public void robotInit() {

    }

    @Override
    public void robotPeriodic() {


    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {




    }

    @Override
    public void autonomousInit() {
        thingy.init();
        SmartDashboard.putNumber("AutoRoutine", 1);
    }

    @Override
    public void autonomousPeriodic() {

        thingy.run(robotHardware);
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {

        if (leftJoystick.getRawButton(2)) {
            robotHardware.moveForward(.2); /*(0,4)*/
        } else if (leftJoystick.getRawButton(3)) {
            robotHardware.moveBackward(.2); /*(0,4)*/
        } else if (leftJoystick.getRawButton(4)) {
            robotHardware.turnLeft(.2);
        } else if (leftJoystick.getRawButton(5)) {
            robotHardware.turnRight(.2);
        } else {
            double driveJoyStickX = leftJoystick.getX();
            double driveJoyStickY = -leftJoystick.getY();

            double drivePowerLeft = driveJoyStickY + driveJoyStickX;
            double drivePowerRight = driveJoyStickY - driveJoyStickX;

            robotHardware.setDrivePower(drivePowerLeft, drivePowerRight);
        }
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }


}
