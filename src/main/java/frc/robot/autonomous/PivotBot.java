package frc.robot.autonomous;

import frc.robot.autonomous.GenericAuto;
import io.github.pseudoresonance.pixy2api.Pixy2Line;

public class PivotBot extends GenericAuto {

    int midPoint = 40;
    int margin = 1; //set margin of error where it wont move at all (prevents jittering)

    int numTimesNull = 0;

    double turnPower = 0.45;

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
                System.out.println("Null PixyCam Vectors, next auto activated");
                robot.setDrivePower(0,0);
            }
        } else {
            //vec = vectors[0]; //set vec
            numTimesNull = 0; //reset null exit counter
            //System.out.println("Pixy Vector: "+vec.toString());
            //System.out.println("test 1");


            if(robot.pixy.vec.length == 1){
                System.out.println(robot.pixy.vec[0].getX0());
            }


            if(robot.pixy.vec.length != 0 && robot.pixy.vec[0] != null){
                //System.out.println("test 2");
                //which point of vector is higher on screen? get that point's X val
                int topXVal = robot.pixy.vec[0].getX1();
                if (robot.pixy.vec[0].getY0() > robot.pixy.vec[0].getY1()) {
                    topXVal = robot.pixy.vec[0].getX0();
                }

                switch (autoStep) {
                    case 1:
                        double toMove = 0.0;

                        if (topXVal > midPoint + margin) {
                            toMove = midPoint - topXVal;
                            robot.setDrivePower(turnPower,-turnPower);
                        } else if (topXVal < midPoint - margin) {
                            toMove = midPoint - topXVal;
                            robot.setDrivePower(-turnPower,turnPower);
                        } else {
                            autoStep++;
                        }

                        break;

                    case 2:
                        robot.stopDriving();
                        break;
                }
            }


        }
    }
}


