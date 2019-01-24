package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StraightAccel {


    public static int autoStep = -1;
    public static void init() {
        autoStep = -1;
    }

    public static Accel2Pos current = new Accel2Pos();


    public static void run(CaMOElot Robot) {

        switch (autoStep) {

            case -1:
                current.reset();
                Robot.encoderL.reset();
                //SmartDashboard.putNumber("XDisplacementInitial(should be 0): " , current.getDisplacementX());
                autoStep = 0;

            case 0:

                current.update(Robot);

                Robot.moveForward(.25);
                if (Robot.encoderL.get() >= 2688) {
                    autoStep = 1;
                }

                break;
            case 1:

                Robot.stopEverything();

                SmartDashboard.putNumber("Meas Pts: ", current.getMeasPts());
                SmartDashboard.putNumber("XDisplacement: ", current.getDisplacementX());
                SmartDashboard.putNumber("YDisplacement: ", current.getDisplacementY());
                SmartDashboard.putNumber("ZDisplacement: ", current.getDisplacementZ());
        }
    }
}