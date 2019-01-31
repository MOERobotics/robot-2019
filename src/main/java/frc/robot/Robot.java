/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.kauailabs.navx.frc.*;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.links.SPILink;


public class Robot extends TimedRobot {
    TalonSRX testTalon;
    AHRS navx;
    //pixy line-detection camera
    private SPILink pixySPI = new SPILink();
    private Pixy2 pixyCam = Pixy2.createInstance(pixySPI);
    private long updateNum;


    @Override
    public void robotInit() {
        pixyCam.init();
    }

    @Override
    public void robotPeriodic() {
        updateNum++;
        if (updateNum % 180 < 90) {
            pixyCam.setLamp((byte)126, (byte)126);
        } else {
            pixyCam.setLamp((byte)0, (byte)0);
        }

        pixyCam.setLED((int)(updateNum / 2) % 255, 0, 0);
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }


}
