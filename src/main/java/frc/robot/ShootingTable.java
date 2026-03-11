
package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;

public class ShootingTable
{
    public class Param
    {
        public double speed;
        public double angle;
    }

    public ArrayList<Double> distance;
    public ArrayList<Param> table;

    public ShootingTable()
    {

    }

    public Param getParams(double distance)
    {
        Param params = new Param();
        return params;
    }

    public Param getParams(Translation2d pos)
    {
        Param params = new Param();
        return params;
    }
    // public Pair<double, double> getParams(Translation2d pos)
    // {
    //     Pair<double, double> params = new Pair<double, double>();
    //     return params;
    // }
}