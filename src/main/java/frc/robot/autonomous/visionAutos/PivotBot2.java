package frc.robot.autonomous.visionAutos;

import frc.robot.autonomous.*;

public class PivotBot2 extends GenericAuto {

    int numTimesNull = 0;
    int pixyWait = 0;
    int topXVal;
    int midCounter;

    long currentTime, startTime;
    double drivePower;

    @Override
    public void init() {
        autoStep = 0;
        startTime = System.currentTimeMillis();
        midCounter = 0;
    }

    @Override
    public void run() {//if we get nothing... do nothing.
        //if we only get one vector, don't try to get a second vector

        currentTime = System.currentTimeMillis() - startTime;
        //a1, a2, and lambda defined in GenericAuto
        drivePower = a1 + (a2 * Math.exp( -((double) currentTime/lambda)));

        //Pixy2Line.Vector vec;
        switch (autoStep) {
            case 0:
                robot.shiftLow();
                autoStep++;
                break;

            case 1:
                if (pixyWait < 5) {
                    pixyWait++;
                    break;
                }
                pixyWait = 0;
                if (robot.pixy.vec.length != 1) {
                    //Null counter, if not detecting pixy lines, don't move
                    numTimesNull++;
                    if (numTimesNull > 4) {
                        robot.setDrivePower(0, 0);
                    }
                } else {
                    numTimesNull = 0; //reset null exit counter

                    if (Math.abs(robot.getHeadingDegrees()) < 15) {
                        if (robot.pixy.vec.length != 0 && robot.pixy.vec[0] != null) {
                            //which point of vector is higher on screen? get that point's X val
                            //which point of vector is higher on screen? get that point's X val
                            //topXVal = robot.pixy.vec[0].getX1();
                            topXVal = (int) (1.0*robot.pixy.vec[0].getX1() + 0.0*robot.pixy.vec[0].getX0());
                            if (robot.pixy.vec[0].getY0() < robot.pixy.vec[0].getY1()) {
                                //topXVal = robot.pixy.vec[0].getX0();
                                topXVal = (int) (1.0*robot.pixy.vec[0].getX0() + 0.0*robot.pixy.vec[0].getX1());
                            }
                        }
                    } else {
                        if (robot.pixy.vec.length != 0 && robot.pixy.vec[0] != null) {
                            //which point of vector is higher on screen? get that point's X val
                            topXVal = robot.pixy.vec[0].getX1();
                            if (robot.pixy.vec[0].getY0() < robot.pixy.vec[0].getY1()) {
                                topXVal = robot.pixy.vec[0].getX0();
                            }
                        }
                    }
                }


                if ( (currentTime < 8000) && (Math.abs(topXVal-midPoint) > margin)) {
                    if (topXVal - midPoint > margin) {
                        midCounter = 0;
                        robot.setDrivePower(drivePower, -drivePower);
                    } else if (topXVal - midPoint < -margin) {
                        midCounter = 0;
                        robot.setDrivePower(-drivePower, drivePower);
                    }
                } else {
                    ++midCounter;
                    robot.setDrivePower(0, 0);
                    if ((midCounter>5) || (currentTime >= 8000)) {
                        autoStep++;
                    }
                }
                break;

            case 2:
                robot.stopDriving();
                break;
        }

    }
}
