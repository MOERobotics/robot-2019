package frc.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

public class PIDTestAuto extends GenericAuto{

    double leftPower = 0.0;
    double rightPower = 0.0;

    AHRS navx = new AHRS(SPI.Port.kMXP,(byte) 50);

    long startTime = 0;
    long endTime = 0;

    public PIDTestAuto(){
        SmartDashboard.putNumber("leftPowerIn", 0.0);
        SmartDashboard.putNumber("rightPowerIn", 0.0);
    }

    @Override
    public void init() {
        leftPower = SmartDashboard.getNumber("leftPowerIn", 0.0);
        rightPower = SmartDashboard.getNumber("rightPowerIn", 0.0);
        autoStep = 1;
        startTime = System.currentTimeMillis();
        endTime = startTime + 2000;
    }

    @Override
    public void run() {
        switch(autoStep){
            case 1:
                robot.setDrivePower(leftPower, rightPower);

                if (System.currentTimeMillis() > endTime){
                    autoStep++;
                }
                break;
            case 2:
                robot.stopDriving();
        }
    }
}
