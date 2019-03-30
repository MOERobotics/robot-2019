package frc.robot.autonomous.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.autonomous.GenericAuto;

public class RetractTest extends GenericAuto {
    PIDModule elevatorPID = new PIDModule(0.1, 0.00, 0);
    PIDModule armPID = new PIDModule(1.75e-2,3.0e-3,0);

    double elevatorCorrection;
    double armCorrection;
    double armPowerBias = 0;
    double elevatorDeploy = 37;
    double elevatorFloor = -28.6-3.13;
    //double elevatorBalance = -28;
    double elevatorBalance = 13;
    double armOut = /*40*/25;

    double hab3Height = 350;
    double retractHeight = 120;
    double fullRetractHeight = 5;
    double steadyPower;
    double startPitch;
    double startTime;

    int pulseCounter = 0;

    //pitch positive is up
    @Override
    public void init() {
        lastStep = 14;
        autoStep = 0;
        steadyPower = 0.3;
        habLevel = 3;
    }

    @Override
    public void run() {
        SmartDashboard.putNumber("Climb autoStep: ", autoStep);

        switch(autoStep) {
            /* assuming Alex has rested both arm and elevator on the HAB*/
            case 0:

                //robot.climb(1.0);
                //robot.setDrivePower(steadyPower,steadyPower);

                if(Math.abs(robot.getClimberLEncoderCount()) > fullRetractHeight + 15){
                    robot.climbLDown(1);
                } else if (Math.abs(robot.getClimberLEncoderCount()) < fullRetractHeight + 15
                        && Math.abs(robot.getClimberLEncoderCount()) > fullRetractHeight) {
                    robot.climbLDown(0.2);
                } else {
                    robot.climbLDown(0);
                }

                if(Math.abs(robot.getClimberREncoderCount()) > fullRetractHeight + 15) {
                    robot.climbRDown(1);
                } else if (Math.abs(robot.getClimberREncoderCount()) < fullRetractHeight + 15
                        && Math.abs(robot.getClimberREncoderCount()) > fullRetractHeight) {
                    robot.climbRDown(0.2);
                } else {
                    robot.climbRDown(0);
                }

                if(Math.abs(robot.getClimberLEncoderCount()) <= fullRetractHeight
                        && Math.abs(robot.getClimberREncoderCount()) <= fullRetractHeight){
                    autoStep++;
                    robot.climb(0);
                    robot.setDrivePower(0,0);
                    startTime = System.currentTimeMillis();
                }
                break;

            case 1:

                if (System.currentTimeMillis() - startTime > 3000) {
                    robot.setDrivePower(0, 0);
                    //autoStep++;
                } else {
                    robot.setDrivePower(steadyPower, steadyPower);
                }
                break;


        }
    }
}
