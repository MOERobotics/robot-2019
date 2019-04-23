package frc.robot.autonomous.sandstorm;

import frc.robot.autonomous.GenericAuto;

public class DoNothingAuto extends GenericAuto {

    @Override
    public void init() {
        autoStep = 0;
    }

    @Override
    public void run() {
        switch(autoStep) {
            case 0:
                robot.setDrivePower(0, 0);
                break;

        }
    }
}
