package frc.robot.autonomous.test;

import frc.robot.PIDModule;
import frc.robot.autonomous.GenericAuto;

public class Cargo3 extends GenericAuto {

    PIDModule elevatorPID = new PIDModule(0.1, 0.00, 0);
    PIDModule armPID = new PIDModule(1.75e-2,3.0e-3,0);
    double armOut = /*21.3*/ 53;
    double elevatorFloor = -28.6-3.13;
    boolean armControlling = false;
    boolean elevatorControlling = false;
    double armCorrection;
    double elevatorCorrection;

    @Override
    public void init() {
        //elevatorPID.resetError();
        //armPID.resetError();
        //robot.resetArmPosition();
        //robot.resetElevatorPosition();
        armControlling = false;
        elevatorControlling = false;
    }

    @Override
    public void run() {

        if (!armControlling) {
            if (Math.abs(robot.getArmEncoderCount() - armOut) < 5){
                armControlling=true;
                armPID.resetError();
            }
            else if (robot.getArmEncoderCount()<armOut)
            {
                robot.driveArm(0.4);
            }
            else
            {
                robot.driveArm(-0.4);
            }
        }
        else
        {
            armPID.setHeading(robot.getArmEncoderCount()  - armOut);
            armCorrection = armPID.getCorrection();

            robot.driveArm(armCorrection);

            if (!elevatorControlling) {
                if (Math.abs(robot.getElevatorEncoderCount() - elevatorFloor) < 5) {
                    elevatorControlling = true;
                    elevatorPID.resetError();
                } else if (robot.getElevatorEncoderCount()<elevatorFloor)
                {
                    robot.driveElevator(0.5);
                }
                else
                {
                    robot.driveElevator(-0.5);
                }
            }
            else
            {
                elevatorPID.setHeading(robot.getElevatorEncoderCount()  - elevatorFloor);
                elevatorCorrection = elevatorPID.getCorrection();

                robot.driveElevator(elevatorCorrection);

            }

        }
    }
}
