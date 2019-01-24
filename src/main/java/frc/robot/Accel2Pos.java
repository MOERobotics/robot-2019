package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Accel2Pos {
    static final int MaxPts  = 500;
    int MeasPts = 0;

    double[] ax = new double[MaxPts];
    double[] ay = new double[MaxPts];
    double[] az = new double[MaxPts];

    double TimeStep = 0.02;

    public void Accel2Pos(){}

    public void reset() {
        MeasPts = 0;
    }

    public int getMeasPts() {
        return MeasPts;
    }

    public void update(GenericRobot robot) {
        if (MeasPts < MaxPts) {
            ax[MeasPts] = robot.getAccelX();
            ay[MeasPts] = robot.getAccelY();
            az[MeasPts] = robot.getAccelZ();

            MeasPts += 1;
        }
    }

    public double getDisplacementX()
    {
        int j;
        double[] v = new double[MaxPts];
        double[] p = new double[MaxPts];
        double cumsum;
        v[0] = 0;
        for (j=1; j<MeasPts; ++j)
        {
            v[j] = v[j-1] + 0.5*(ax[j-1]+ax[j])*TimeStep;
        }

        p[0] = 0;
        for (j=1; j<MeasPts; ++j)
        {
            p[j] = p[j-1] + 0.5*(v[j-1]+v[j])*TimeStep;
        }
        return p[MeasPts-1];
    }

    public double getDisplacementY()
    {
        int j;
        double[] v = new double[MaxPts];
        double[] p = new double[MaxPts];
        double cumsum;

        v[0] = 0;

        for (j=1; j<MeasPts; ++j)
        {
            v[j] = v[j-1] + 0.5*(ay[j-1]+ay[j])*TimeStep;
        }

        p[0] = 0;
        for (j=1; j<MeasPts; ++j)
        {
            p[j] = p[j-1] + 0.5*(v[j-1]+v[j])*TimeStep;
        }
        return p[MeasPts - 1];
    }

    public double getDisplacementZ()
    {
        int j;
        double[] v = new double[MaxPts];
        double[] p = new double[MaxPts];
        double cumsum;

        v[0] = 0;

        for (j=1; j<MeasPts; ++j)
        {
            v[j] = v[j-1] + 0.5*(az[j-1]+az[j])*TimeStep;
        }

        p[0] = 0;
        for (j=1; j<MeasPts; ++j)
        {
            p[j] = p[j-1] + 0.5*(v[j-1]+v[j])*TimeStep;
        }
        return p[MeasPts-1];
    }
}
