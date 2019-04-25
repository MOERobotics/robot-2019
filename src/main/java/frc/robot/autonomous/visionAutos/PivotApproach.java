package frc.robot.autonomous.visionAutos;

import frc.robot.autonomous.GenericAuto;

public class PivotApproach extends GenericAuto {

    int numTimesNull = 0;
    long startTime;
    int pixyWait = 0;
    int counter = 0;

    @Override
    public void init() {
        autoStep = 1;
    }

    @Override
    public void run() {//if we get nothing... do nothing.
        //if we only get one vector, don't try to get a second vector
        //if we get two vectors, proceed

        //Pixy2Line.Vector vec;
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

            if(robot.pixy.vec.length != 0 && robot.pixy.vec[0] != null){
                //System.out.println("test 2");
                //which point of vector is higher on screen? get that point's X val
                int topXVal = robot.pixy.vec[0].getX1();
                if (robot.pixy.vec[0].getY0() < robot.pixy.vec[0].getY1()) {
                    topXVal = robot.pixy.vec[0].getX0();
                }

                switch (autoStep) {
                    case 1:
                        if(pixyWait < 5){ pixyWait++; break; }
                        pixyWait = 0;

                        if (topXVal > midPoint + margin) {
                            counter = 0;
                            if (topXVal > midPoint + biggerMargin) {
                                robot.setDrivePower(higherTurnPower,-higherTurnPower);
                            } else {
                                robot.setDrivePower(turnPower, -turnPower);
                            }
                        } else if (topXVal < midPoint - margin) {
                            counter = 0;
                            if (topXVal < midPoint - biggerMargin) {
                                robot.setDrivePower(-higherTurnPower,higherTurnPower);
                            } else {
                                robot.setDrivePower(-turnPower, turnPower);
                            }
                        } else {
                            if (counter > 5) {
                                counter = 0;
                                autoStep++;
                            }
                            counter++;
                        }

                        break;

                    case 2:
                        robot.spearOut();
                        robot.setDrivePower(0.3,0.3);
                        if(robot.lidar[0] < 500){
                            autoStep++;
                            startTime = System.currentTimeMillis();
                        }
                        break;

                    case 3:
                        robot.setDrivePower(0.2,0.2);
                        if(System.currentTimeMillis() - 500 > startTime){
                            autoStep++;
                        }
                        break;

                    case 4:
                        robot.stopDriving();
                        break;
                }
            }


        }
    }
}


