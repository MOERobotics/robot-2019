/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import io.github.pseudoresonance.pixy2api.Pixy2;

import io.github.pseudoresonance.pixy2api.Pixy2Line;
import io.github.pseudoresonance.pixy2api.links.SPILink;



public class Robot extends TimedRobot {

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
        updateNum++;

        pixyCam.getLine().getAllFeatures();
        Pixy2Line.Vector[] vec = pixyCam.getLine().getVectors();

        if(vec == null){ return; }
        for(int i=0;i<vec.length;i++){
            System.out.println(vec[i].toString());
        }
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }


}
