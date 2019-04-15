package frc.robot.autonomous.unused;

import frc.robot.autonomous.GenericAuto;

public class ArmAuto extends GenericAuto {

    @Override
    public void init() {
        autoStep = 0;
    }

    @Override
    public void run() {
        //arm up 4 ticks
        //down 3 ticks

        switch(autoStep) {
            case 0:
                if (robot.getArmEncoderCount() > 20) {
                    robot.driveArm(0);
                    autoStep++;
                } else robot.driveArm(0.5);
                break;
            case 1:
                if (robot.getArmEncoderCount() < 5) {
                    robot.driveArm(0);
                } else robot.driveArm(-0.5);
                break;
        }
    }
}
