package frc.robot;


public class TestSideLidar extends GenericAuto {
    PIDModule MOErioAuto = new PIDModule(0.06, 0.001, 0);
    double correction = 0;

    int counter = 0;
    double beforeLidar;
    double flatTol = 25;
    double maxRange = 1000;

    @Override
    public void init() {
        robot.resetYaw();
        MOErioAuto.resetError();

        counter = 0;
        autoStep = 1;
        beforeLidar = robot.lidar[0];
    }

    @Override
    public void run() {

        switch(autoStep){
            case 1:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(0.2*(1 + correction),0.2*(1 - correction));

                if(robot.lidar[0] < maxRange){
                    if(Math.abs(robot.lidar[0]-beforeLidar) < flatTol){
                        counter++;
                    }
                    else {
                        beforeLidar = robot.lidar[0];
                        counter = 0;
                    }
                }

                if(counter == 4){
                    autoStep++;
                }
                break;
            case 2:
                robot.stopDriving();
                break;
        }

    }


}

