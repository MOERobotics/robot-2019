package frc.robot;


public class TestSideLidar extends GenericAuto {
    PIDModule MOErioAuto = new PIDModule(0.06, 0.001, 0);
    double correction = 0;

    int counter = 0;
    int beforeLidar;
    int flatTol = 25;
    int maxRange = 1000;

    int[] lidarHistory = new int[8];
    double [] lidarAvg = new double[5];

    public void shift() {
        int i;

        for (i=0; i<7; ++i) {
            lidarHistory[i] = lidarHistory[i+1];
        }
    }

    public void avg() {
        int i,j;

        for (i=0; i<5; ++i){
         lidarAvg[i] = 0;
         for (j=0; j<3; ++j) {
             lidarAvg[i] += lidarHistory[i+j];
         }
         lidarAvg[i] /= 3.;
        }
    }

    @Override
    public void init() {
        robot.resetYaw();
        MOErioAuto.resetError();

        counter = 0;
        autoStep = 1;
        beforeLidar = robot.lidar[0];
        lidarHistory[0] = robot.lidar[0];
        lidarHistory[1] = robot.lidar[0];
        lidarHistory[2] = robot.lidar[0];
        lidarHistory[3] = robot.lidar[0];
        lidarHistory[4] = robot.lidar[0];
        lidarHistory[5] = robot.lidar[0];
        lidarHistory[6] = robot.lidar[0];
        lidarHistory[7] = robot.lidar[0];
    }

    @Override
    public void run() {

        switch(autoStep){
            case 1:
                MOErioAuto.setHeading(robot.getHeadingDegrees());
                correction = MOErioAuto.getCorrection();

                robot.setDrivePower(0.2*(1 + correction),0.2*(1 - correction));

                shift();
                lidarHistory[7] = robot.lidar[0];
                avg();

                if(robot.lidar[0] < maxRange){
                    if (
                            (Math.abs(lidarAvg[0]-lidarAvg[4]) < flatTol) &&
                            (Math.abs(lidarAvg[0]-lidarAvg[3]) < flatTol) &&
                            (Math.abs(lidarAvg[0]-lidarAvg[2]) < flatTol)
                    ) {
                        autoStep++;
                    }
                    else {
                        // = robot.lidar[0];
                        counter = 0;
                    }
                }

                //if(counter == 4){
                  //  autoStep++;
                //}
                break;
            case 2:
                robot.stopDriving();
                break;
        }

    }


}

