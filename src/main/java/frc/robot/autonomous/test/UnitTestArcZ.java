
//Unit testing for a controlled arc

//This should carry the robot through a 90 arc having an outer radius of 66.5 inches.

package frc.robot.autonomous.test;

        import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
        import frc.robot.PIDModule;
        import frc.robot.autonomous.GenericAuto;


public class UnitTestArcZ extends GenericAuto {
    PIDModule MOErioAuto = new PIDModule(0.06, 0.001, 0);
    double LouWizardry;
    double correction;
    long startTime = 0;
    double z = 1.33;

    @Override
    public void init() {
        autoStep = -2;
        robot.resetDriveEncoders();
        robot.resetYaw();
        MOErioAuto.resetError();
        MOErioAuto.setHeading(0);
        startTime = System.currentTimeMillis();
    }

    @Override
    public void printSmartDashboard() {

        SmartDashboard.putNumber("Error: ", MOErioAuto.getInput());
        SmartDashboard.putNumber("Correction: ", MOErioAuto.getCorrection());
        SmartDashboard.putNumber("kP: ", MOErioAuto.pidController.getP());
        SmartDashboard.putNumber("kI: ", MOErioAuto.pidController.getI());
        SmartDashboard.putNumber("kD: ", MOErioAuto.pidController.getD());
        SmartDashboard.putNumber("Current Time: ", System.currentTimeMillis());
        SmartDashboard.putNumber("Abs Left", Math.abs(robot.getDistanceLeftInches()));
        SmartDashboard.putNumber("Abs Right", Math.abs(robot.getDistanceRightInches()));
    }

    public void run() {
        double leftDistance = Math.abs(robot.getDistanceLeftInches());
        double rightDistance = Math.abs(robot.getDistanceRightInches());

        LouWizardry = leftDistance - rightDistance * z;

        switch (autoStep) {
            case -2:
                MOErioAuto.resetError();
                robot.resetYaw();

                if (System.currentTimeMillis() >= startTime + 100) {
                    MOErioAuto.resetError();
                    robot.resetYaw();
                    autoStep = -1;
                }
                break;
            case -1:
                MOErioAuto.setHeading(LouWizardry);
                correction = MOErioAuto.getCorrection();

                //correction negative, left motor decrease, correction positive, left motor power increase
                robot.setDrivePower((0.5)*(1 + correction),(0.3)*(1 - correction));

                if (leftDistance >= 48) {
                    autoStep++;
                    MOErioAuto.resetError();
                    robot.resetDriveEncoders();
                }
                break;
            case 0:
                LouWizardry = leftDistance - rightDistance / z;
                MOErioAuto.setHeading(LouWizardry);
                correction = MOErioAuto.getCorrection();

                //correction negative, left motor decrease, correction positive, left motor power increase
                robot.setDrivePower((0.3)*(1 + correction),(0.5) *(1 - correction));

                if (leftDistance >= 48 / z) {
                    autoStep++;
                }
                break;
            case 1:
                robot.stopDriving();
                break;
        }
    }
}
