package frc.robot.autonomous.visionAutos;

import frc.robot.autonomous.GenericAuto;

public class PivotApproach extends GenericAuto {

    int midPoint = 34;
    int margin = 1; //set margin of error where it wont move at all (prevents jittering)
    int biggerMargin = 8;

    int numTimesNull = 0;

    double turnPower = /*0.2*/0.28;
    double higherTurnPower = /*0.25*/0.33;

    long startTime;

    int pixyWait = 0;

    @Override
    public void init() {
        autoStep = 1;
    }

    @Override
    public void run() {//if we get nothing... do nothing.
        //if we only get one vector, don't try to get a second vector
        //if we get two vectors, proceed
        /*
        if(robot.pixy.vec){
            Pixy2Line.Vector vec = robot.pixyXY[0];
            if(vec != null){
                System.out.println(vec.toString());
            }else{
                System.out.println("Vec was null");
            }
        }else{
            System.out.println("No vecs found");
        }
        */

        //Pixy2Line.Vector[] vectors = robot.pixyXY;

        //Pixy2Line.Vector vectors[] = new Pixy2Line.Vector[1];
        //System.out.println(vectors.length);

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
            //System.out.println("Pixy Vector: "+vec.toString());
            //System.out.println("test 1");


            /*if(robot.pixy.vec.length == 1){
                System.out.println(robot.pixy.vec[0].getX0());
            }*/


            if(robot.pixy.vec.length != 0 && robot.pixy.vec[0] != null){
                //System.out.println("test 2");
                //which point of vector is higher on screen? get that point's X val
                int topXVal = robot.pixy.vec[0].getX1();
                if (robot.pixy.vec[0].getY0() < robot.pixy.vec[0].getY1()) {
                    topXVal = robot.pixy.vec[0].getX0();
                }

                switch (autoStep) {
                    case 1:
                        double toMove = 0.0;

                        if(pixyWait < 5){ pixyWait++; break; }
                        pixyWait = 0;

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


