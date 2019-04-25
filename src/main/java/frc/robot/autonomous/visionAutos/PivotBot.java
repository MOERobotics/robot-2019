package frc.robot.autonomous.visionAutos;

import frc.robot.autonomous.GenericAuto;
import io.github.pseudoresonance.pixy2api.Pixy2Line;

public class PivotBot extends GenericAuto {

    int numTimesNull = 0;
    int pixyWait = 0;
    int topXVal;
    int counter = 0;

    //boolean wasHighGear;

    @Override
    public void init() {
        autoStep = 1;
    }

    @Override
    public void run() {//if we get nothing... do nothing.
        //if we only get one vector, don't try to get a second vector

        //Pixy2Line.Vector vec;
        switch (autoStep) {
            /*case 0:
                wasHighGear = robot.getShifterSolenoidState();
                robot.shiftLow();
                autoStep++;
                break;*/

            case 1:
                if(pixyWait < 5){ pixyWait++; break; }
                pixyWait = 0;
                if (robot.pixy.vec.length != 1) {
                    //Null counter, if not detecting pixy lines, don't move
                    numTimesNull++;
                    if (numTimesNull > 4){
                        robot.setDrivePower(0, 0);
                    }
                } else {
                    numTimesNull = 0; //reset null exit counter
                    if(robot.pixy.vec.length != 0 && robot.pixy.vec[0] != null) {
                        //which point of vector is higher on screen? get that point's X val
                        topXVal = robot.pixy.vec[0].getX1();
                        if (robot.pixy.vec[0].getY0() < robot.pixy.vec[0].getY1()) {
                            topXVal = robot.pixy.vec[0].getX0();
                        }
                    }

                    if (topXVal > midPoint + margin) {
                        counter = 0;
                        if (topXVal > midPoint + biggerMargin) {
                            robot.setDrivePower(higherTurnPower,-higherTurnPower);
                            //robot.setDrivePower(higherTurnPower, 0);
                        } else {
                            robot.setDrivePower(turnPower, -turnPower);
                            //robot.setDrivePower(turnPower, 0);
                        }
                    } else if (topXVal < midPoint - margin) {
                        counter = 0;
                        if (topXVal < midPoint - biggerMargin) {
                            robot.setDrivePower(-higherTurnPower,higherTurnPower);
                            //robot.setDrivePower(-higherTurnPower, 0);
                        } else {
                            robot.setDrivePower(-turnPower, turnPower);
                            //robot.setDrivePower(-turnPower, 0);
                        }
                    } else {
                        robot.setDrivePower(0, 0);
                        counter++;
                        if (counter > 4) {
                            autoStep++;
                            counter = 0;
                        }
                    }
                }
                break;

            case 2:
                robot.stopDriving();
                //autoStep++;
                break;

            /*case 3:
                robot.shiftDrive(wasHighGear);
                break;*/

        }

    }
}


