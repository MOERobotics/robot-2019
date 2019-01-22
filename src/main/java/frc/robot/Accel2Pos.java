package frc.robot;

import frc.robot.genericrobot.GenericRobot;

import com.kauailabs.navx.frc.AHRS;

import java.sql.Time;

public class Accel2Pos {
    static int MaxPts  = 500;
    int MeasPts = 0;

    double[] ax = new double[MaxPts];
    double[] ay = new double[MaxPts];
    double[] az = new double[MaxPts];

    double TimeStep = 0.02;

    public void reset() {
        MeasPts = 0;
    }

    public void update(GenericRobot Robot) {
        if (MeasPts < MaxPts) {
            ax[MeasPts] = Robot.getAccelX();
            ay[MeasPts] = Robot.getAccelY();
            az[MeasPts] = Robot.getAccelZ();
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
        return p[MeasPts];
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
        return p[MeasPts];
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
        return p[MeasPts];
    }
}
