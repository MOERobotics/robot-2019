package frc.robot.autonomous.visionAutos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.GenericAuto;
import frc.robot.PIDModule;


public class PivotApproach2 extends GenericAuto {

    int numTimesNull = 0;
    int pixyWait = 0;
    int topXVal;
    int midCounter;

    long currentTime, startTime;
    double drivePower;

    double PivotHeading;
    double correction;

    PIDModule MOErioAuto = new PIDModule(0.06, 0.001, 0);

    @Override
    public void init() {
        autoStep = 0;
        startTime = System.currentTimeMillis();
        midCounter = 0;
    }

    @Override
    public void run() {//if we get nothing... do nothing.
        //if we only get one vector, don't try to get a second vector
        //if we get two vectors, proceed

        SmartDashboard.putNumber("Approach autostep", autoStep);

        currentTime = System.currentTimeMillis() - startTime;
        drivePower = a1 + (a2 * Math.exp( -((double) currentTime/lambda)));

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
                            topXVal = (int) (0.8*robot.pixy.vec[0].getX1() + 0.2*robot.pixy.vec[0].getX0());
                            if (robot.pixy.vec[0].getY0() < robot.pixy.vec[0].getY1()) {
                                topXVal = (int) (0.8*robot.pixy.vec[0].getX0() + 0.2*robot.pixy.vec[0].getX1());
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
                    PivotHeading = robot.getHeadingDegrees();

                    if ((midCounter>5) || (currentTime >= 8000)) {
                        MOErioAuto.resetError();
                        autoStep++;
                    }
                }
                break;

                case 2:
                    robot.spearOut();
                    MOErioAuto.setHeading(robot.getHeadingDegrees()-PivotHeading);
                    correction = MOErioAuto.getCorrection();
                    robot.setDrivePower((0.3) * (1 + correction), (0.3) * (1 - correction));

                    //robot.setDrivePower(0.3,0.3);
                    if(robot.lidar[0] < 500){
                        autoStep++;
                        startTime = System.currentTimeMillis();
                    }
                    break;

                case 3:
                    MOErioAuto.setHeading(robot.getHeadingDegrees()-PivotHeading);
                    correction = MOErioAuto.getCorrection();
                    robot.setDrivePower((0.1) * (1 + correction), (0.1) * (1 - correction));
                    //robot.setDrivePower(0.2,0.2);
                    if(System.currentTimeMillis() - 250 > startTime){
                        autoStep++;
                    }
                    break;

                case 4:
                    robot.stopDriving();
                    break;
            }
        }
}
