package frc.robot.autonomous.visionAutos;
import frc.robot.autonomous.*;

//NEEDS TESTING
public class PiCamAlign extends GenericAuto {
    int target = 240; //check

    @Override
    public void init() {
        autoStep = 0;
    }

    @Override
    public void run() {

        switch (autoStep) {
            case 0:
                if (robot.piXY[0] != -1 && robot.piXY[1] != -1) {
                    if (robot.piXY[0] > target + margin) {
                        if (robot.piXY[0] > target + biggerMargin) {
                            robot.setDrivePower(higherTurnPower,-higherTurnPower);
                        } else {
                            robot.setDrivePower(turnPower, -turnPower);
                        }
                    } else if (robot.piXY[0] < target - margin) {
                        if (robot.piXY[0] < target - biggerMargin) {
                            robot.setDrivePower(-higherTurnPower,higherTurnPower);
                        } else {
                            robot.setDrivePower(-turnPower, turnPower);
                        }
                    } else {
                        autoStep++;
                    }
                }
                break;


            case 1:
                robot.stopDriving();
                break;
        }

    }
}
