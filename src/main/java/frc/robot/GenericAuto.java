package frc.robot;

import frc.robot.genericrobot.GenericRobot;

public abstract class GenericAuto {
    public int autoStep = 0;
    public abstract void init();
    public abstract void run();
    public GenericRobot robot;
}
