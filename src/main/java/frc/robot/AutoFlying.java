package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*WHAT IS PITCH???*/

public class AutoFlying extends GenericAuto {
    double hab2Height;
    double hab3Height;
    double steadyPower;
    double startPitch;
    double startTime;
    double ClimbEncoderOrigin;


    //pitch positive is up
    @Override
    public void init() {
        hab2Height = 20; /*these need to be fixed!!!*/
        hab3Height = 30;
        startPitch = robot.getPitchDegrees();
        autoStep = 0;
        steadyPower = 0.3;
    }

    public void setClimbEncoderOrigin(double climbEncoderOrigin) {
        this.ClimbEncoderOrigin = climbEncoderOrigin;
    }

    @Override
    public void run() {

        switch(autoStep) {
            /*drive the arm and elevator DOWN onto the HAB level and climb up until we reach the specified height,
             * assuming Alex has rested them both on it (if he hasn't then we're--*/
            case 0:
                // Anthony thinks this is a good idea.
                // Let's start with something simpler.
                //robot.driveArm(-steadyPower);
                //robot.driveElevator(-steadyPower);
                robot.climb(-1.0);

                if ((robot.getClimberLEncoderCount() - ClimbEncoderOrigin) >= hab2Height && habLevel == 2) {
                    autoStep++;
                } else if ((robot.getClimberLEncoderCount() - ClimbEncoderOrigin) >= hab3Height && habLevel == 3) {
                    autoStep++;
                } else if (habLevel != 3 || habLevel != 2) {
                    autoStep = 10;
                }
                break;
            /*extend mini spacer air cylinders*/
            case 1:
                robot.footSpacerCylinder(true);
                startTime = System.currentTimeMillis();
                autoStep++;
                break;
            /*slide robot forward and also drive forward until it has fallen forward (either it's been doing that long
             * enough or pitch says so*/
            case 2:
                //robot.climbPushForwardz(DoubleSolenoid.Value.kForward);
                robot.LinearSlider(DoubleSolenoid.Value.kForward);
                robot.setDrivePower(steadyPower, steadyPower);
                autoStep++;
                break;
            /*drive forward for one second*/
            case 3:
                robot.setDrivePower(steadyPower,steadyPower);

                if(System.currentTimeMillis()-startTime < 2000) {
                    autoStep++;
                }
                break;
            /*retract the climbers until pitch is leaning only 5 degrees towards the driver station*/
            case 4:
                robot.climb(0.2);

                if (robot.getPitchDegrees()-startPitch > - 5) {
                    autoStep++;
                    startTime = System.currentTimeMillis();
                }
                break;
            /*drive forward for one second*/
            case 5:
                robot.setDrivePower(steadyPower,steadyPower);

                if (System.currentTimeMillis() >= startTime + 1000) {
                    autoStep++;
                }
                break;
            /*raise arm to somewhere between horizontal and 30 degrees below it*/
            case 6:
                robot.driveArm(0.3);
                //how do i know where the arm even is though
                if(robot.getArmEncoderCount() >= 12){
                    autoStep++;
                }
                break;
            /*retract climber until we are level*/
            case 7:
                robot.climb(0.2);

                if (Math.abs(robot.getPitchDegrees()-startPitch) < 1) {
                    autoStep++;
                    startTime = System.currentTimeMillis();
                }
                break;
            /*continue retracting for another second*/
            case 8:
                robot.climb(0.2);

                if (System.currentTimeMillis() >= startTime + 1000) {
                    autoStep++;
                }
                break;
            /*retract those mini air cylinders*/
            case 9:
                robot.footSpacerCylinder(false);
                autoStep++;
                break;
            case 10:
                robot.resetDriveEncoders();
                //PIDTune to 0 distance on the ENCODERS
                robot.stopEverything();
                break;
        }
    }
}
