package frc.robot;

public class DoNothingAuto extends GenericAuto{

    @Override
    public void init() {

    }

    @Override
    public void run() {
        switch(autoStep) {
            case 1:
                robot.setDrivePower(0, 0);
                break;

        }
    }
}