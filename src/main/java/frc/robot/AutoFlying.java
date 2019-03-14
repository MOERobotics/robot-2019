package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class AutoFlying extends GenericAuto {
    double climbOrigin;
    double hab2Height;
    double hab3Height;
    double steadyPower;
    double startPitch;
    double startTime;
    int habLevel;

    @Override
    public void init() {
        climbOrigin = robot.getClimbOrigin();
        hab2Height;
        hab3Height;
        startPitch = robot.getPitchDegrees();
        startTime = System.currentTimeMillis();
        autoStep = 0;
    }

    @Override
    public void run() {


        switch(autoStep) {
            case 0:
                robot.driveArm(steadyPower);
                robot.driveElevator(steadyPower);
                robot.climb(ihatethis);

                if(robot.getClimberLEncoderCount() >= hab2Height && habLevel == 2) {
                    autoStep++;
                } else if (robot.getClimberLEncoderCount() >= hab3Height && habLevel == 3){
                    autoStep++;
                }
                break;
            case 1:
                //robot.extendCylinders();
                //if (extendCylinders == max) {
                //  autoStep++;
                //}
                break;
            case 2:
                robot.climbPushForwardz(DoubleSolenoid.Value.kForward);
                robot.climb2(DoubleSolenoid.Value.kForward);
                robot.setDrivePower(steadyPower, steadyPower);

                if (robot.getPitchDegrees() < startPitch - spoopynumber) {
                    autoStep++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 3:
                robot.setDrivePower(0.5);

                if (System.currentTimeMillis() >= startTime + 1000) {
                    autoStep++;
                }
                break;
            case 4:
                robot.climb(retractpower);

                if (robot.getPitchDegrees() < 5) {
                    autoStep++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 5:
                robot.setDrivePower(0.5);

                if (System.currentTimeMillis() >= startTime + 1000) {
                    autoStep++;
                }
                break;
            case 6:
                robot.climb(retractpower);

                if(robot.getPitchDegrees() < startPitch - 5 || robot.getPitchDegrees() > startPitch + 5) {
                    autoStep++;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 7:
                robot.climb(retractpower);

                if (System.currentTimeMillis() >= startTime + 1000) {
                    autoStep++;
                }
                break;
            case 8:
                //robot.retractCylinders();
                //if (robot.retractCylinders() == max) {
                //  autoStep++;
                //}
                break;
            case 9:
                robot.stopEverything();
        }
    }
}
