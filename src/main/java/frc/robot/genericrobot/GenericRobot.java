package frc.robot.genericrobot;
/*code*/

public abstract class GenericRobot {

    double leftPower;
    double rightPower;

    public abstract double getDistanceLeftInches();
    public abstract double getDistanceRightInches();
    public abstract void resetDistance();

    public abstract double getHeadingDegrees();
    public abstract void resetDegrees();

    protected abstract void setDrivePowerInternal(double leftMotor, double rightMotor);
    public final void setDrivePower(double leftMotor, double rightMotor) {
        leftPower = leftMotor;
        rightPower = rightMotor;
        setDrivePowerInternal(leftMotor, rightMotor);
    }
    public final void moveForward(double motorPower){
        setDrivePower(motorPower, motorPower);
    }
    public final void moveBackward(double motorPower){
        setDrivePower(-motorPower, -motorPower);
    }
    public double getDrivePowerLeft(){
        return leftPower;
    }
    public double getDrivePowerRight(){
        return rightPower;
    }

    public void turnLeft(double motorPower){
        setDrivePowerInternal(-motorPower, motorPower);
    }
    public void turnRight(double motorPower){
        setDrivePowerInternal(motorPower, -motorPower);
    }

    /*Two types of turning, still and hooked. Add hooked later*/

    public final void stopDriveMotors(){
        setDrivePower(0,0);
    }

    /*turret or waist functions*/
    public abstract void turnTurretLeft();
    public abstract void turnTurretRight();
    public abstract double getTurretDegrees();
    public abstract void stopTurret();

    /*elevator or lift or arm functions*/
    public abstract void upArm();
    public abstract void downArm();
    public abstract double getArmHeight();
    public abstract void stopArm();

    /*elbow or i don't remember the CAD term for it functions*/
    public abstract void rotateElbowUp();
    public abstract void rotateElbowDown();
    public abstract double getElbowDegrees();
    public abstract void stopElbow();

    /*wrist functions*/
    public abstract void inWrist();
    public abstract void outWrist();
    public abstract void stopWrist();

    /*spear or harpoon functions*/
    public abstract void inSpear();
    public abstract void outSpear();
    public abstract void openSpear();
    public abstract void closeSpear();

    /*hi*/
    public final void stopEverything(){
        stopDriveMotors();
    }

    public abstract double getAccelX();
    public abstract double getAccelY();
    public abstract double getAccelZ();
}

