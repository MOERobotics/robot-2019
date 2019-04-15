package frc.robot.autonomous;

import frc.robot.autonomous.GenericAuto;
import io.github.pseudoresonance.pixy2api.Pixy2Line;

public class PivotBot extends GenericAuto {

    int midPoint = 34;
    int margin = 1; //set margin of error where it wont move at all (prevents jittering)
    int biggerMargin = 8;

    int numTimesNull = 0;
    int pixyWait = 0;

    double turnPower = /*0.2*/0.4;
    double higherTurnPower = /*0.25*/0.45;

    long startTime;

    int topXVal;

    @Override
    public void init() {
        autoStep = 1;
    }

    @Override
    public void run() {//if we get nothing... do nothing.
        //if we only get one vector, don't try to get a second vector

        //Pixy2Line.Vector vec;
        switch (autoStep) {
            case 1:
                double toMove = 0.0;

                        /*if (topXVal > midPoint + margin) {
                            if (topXVal > midPoint + biggerMargin) {
                                robot.setDrivePower(higherTurnPower,-higherTurnPower);
                            } else {
                                robot.setDrivePower(turnPower, -turnPower);
                            }
                        } else if (topXVal < midPoint - margin) {
                            if (topXVal < midPoint - biggerMargin) {
                                robot.setDrivePower(-higherTurnPower,higherTurnPower);
                            } else {
                                robot.setDrivePower(-turnPower, turnPower);
                            }
                        } else {
                            autoStep++;
                        }

                        break;*/

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

                        //If top vec coord is to far to right
                    /*
                    if (topXVal > midPoint + margin) {
                        //logic here: if we're closer to the line (vertically speaking), the horizontal margin will be greater.
                        if (topXVal > midPoint + biggerMargin) {
                            robot.setDrivePower(turnPower, 0);
                        } else {
                            robot.setDrivePower(higherTurnPower, 0);
                        }
                    } else if (topXVal < midPoint - margin) {
                        if (topXVal < midPoint - biggerMargin) { //big margin because line moves farther, the closer robot is
                            robot.setDrivePower(0, turnPower);
                        } else {
                            robot.setDrivePower(0, higherTurnPower);
                        }
                    }

                    if (Math.abs(topXVal - midPoint) <= margin){
                        autoStep++;
                        startTime = System.currentTimeMillis();
                    }
                    */
                    if (topXVal > midPoint + margin) {
                        if (topXVal > midPoint + biggerMargin) {
                            toMove = midPoint - topXVal;
                            robot.setDrivePower(higherTurnPower,-higherTurnPower);
                        } else {
                            robot.setDrivePower(turnPower, -turnPower);
                        }
                    } else if (topXVal < midPoint - margin) {
                        if (topXVal < midPoint - biggerMargin) {
                            toMove = midPoint - topXVal;
                            robot.setDrivePower(-higherTurnPower,higherTurnPower);
                        } else {
                            robot.setDrivePower(-turnPower, turnPower);
                        }
                    } else {
                        autoStep++;
                    }
                }
                break;


            case 2:
                robot.stopDriving();
                break;
        }
        /*
        if (robot.pixy.vec.length != 1) {
            numTimesNull++;
            if (numTimesNull > 4) {
                //autoStep = 2;
                //System.out.println("Null PixyCam Vectors, next auto activated");
                robot.setDrivePower(0,0);
            }
        } else {
            //vec = vectors[0]; //set vec
            numTimesNull = 0; //reset null exit counter

            if(robot.pixy.vec[0] != null){
                Pixy2Line.Vector daVec = robot.pixy.vec[0];
                //which point of vector is higher on screen? get that point's X val
                int topXVal = daVec.getX1();
                if (daVec.getY0() < daVec.getY1()) {
                    topXVal = daVec.getX0();
                }


            }


        }*/
    }
}


