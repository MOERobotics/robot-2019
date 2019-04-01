package frc.robot.autonomous;

import frc.robot.autonomous.GenericAuto;
import io.github.pseudoresonance.pixy2api.Pixy2Line;

public class PivotBot extends GenericAuto {

    int midPoint = 40;
    int margin = 2; //set margin of error where it wont move at all (prevents jittering)

    int numTimesNull = 0;

    @Override
    public void init() {
        autoStep = 1;
    }

    @Override
    public void run() {//if we get nothing... do nothing.
        //if we only get one vector, don't try to get a second vector
        //if we get two vectors, proceed

        Pixy2Line.Vector vec = robot.pixyXY[0];
        if(vec == null){
            numTimesNull++;
            if(numTimesNull > 4){
                autoStep=2;
            }
        }else{
            numTimesNull = 0;
            autoStep = 1;
        }

        Pixy2Line.Vector vec2 = robot.pixyXY[1];
        if(vec2 != null){
            if(vec.getY1() - vec.getY0() < vec2.getY1() - vec2.getY0()){ //Get larger vec (based on length)
                vec = vec2;
            }
        }

        //which point of vector is higher on screen? get that X val
        int topXVal = vec.getX1();
        if(vec.getY0() > vec.getY1()){
            topXVal = vec.getX0();
        }

        switch (autoStep) {
            case 1:
                double toMove = 0.0;

                if(topXVal > midPoint+margin){
                    toMove = midPoint - topXVal;
                }else if(topXVal < midPoint-margin) {
                    toMove = midPoint - topXVal;
                }else{
                    autoStep++;
                }

                robot.turnLeftInplace(toMove/50); //divide by 50 to make it move slow...

                break;

            case 2:
                robot.stopDriving();
                break;
        }


    }
}


