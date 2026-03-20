package frc.robot;

import edu.wpi.first.math.util.Units;

public class RangeTable
{
    public class Range
    {
        public double distance;
        public double speed;
        public double elevation;

        public Range(double distance, double speed, double elevation)
        {
            this.distance = distance;
            this.speed = speed;
            this.elevation = elevation;
        }
    };

// Elevation
// 4.25 in = 20 => k = 4.7
    private final double hoodCoef = (9.0 - 0.0) / 2.0;  // Position/inch

    private Range[] ranges =
    {
        new Range(Units.inchesToMeters(47.75 / 2 + Constants.Robot.Length / 2 + 8), 2200.0 / 60.0, hoodCoef * 0), // 0
        new Range(Units.inchesToMeters(47.75 / 2 + Constants.Robot.Length / 2 + 60), 2600.0 / 60.0, hoodCoef * 7.0 / 16.0), // 7/16
        new Range(Units.inchesToMeters(47.75 / 2 + Constants.Robot.Length / 2 + 120), 2600.0 / 60.0, hoodCoef * 1.25), // 1.25
        new Range(Units.inchesToMeters(1.4142 * 47.75 / 2 + Constants.Robot.Length / 2 + 152), 3100.0 / 60.0, hoodCoef * 1.5) // 1.5
    };

//2 - -3.45
//1.75 - 4.7
//1.5 - -5.7
//1.25 - -1.2
//1 - -1.5
//0.75 - -2.6
//0.5
//0.25
//0
    private Range[] rangesPrev =
    {
        new Range(17.5, 2350.0 / 60.0, 0), //0
        new Range(47, 2350.0 / 60.0, 4.7), //1
        new Range(77, 2350.0 / 60.0, 7), //1.5
        new Range(107, 2550.0 / 60.0, 10.58), //2.25
        new Range(127, 3030.0 / 60.0, 6.5), //1.75
        new Range(147, 3150.0 / 60.0, 7.6),
        new Range(174, 3350.0 / 60.0, 8.2) //
    };

    public RangeTable()
    {
    }

    public double getSpeedPreset(int preset)
    {
        if (preset > ranges.length)
            preset = ranges.length - 1;
        
        return preset >= 0 ? ranges[preset].speed : 0;
    }

    public double getElevationPreset(int preset)
    {
        if (preset > ranges.length)
            preset = ranges.length - 1;
        
        return preset >= 0 ? ranges[preset].elevation : 0;
    }

    public double lerp(double a, double b, double x)
    {
        return a * (1.0 - x) + x * b;
    }

    public Range getRange(double distance)
    {
        if (ranges.length == 0)
            return null;

        int i = 0;
        while (i < ranges.length && distance <= ranges[i].distance)
            ++i;

        if (i == 0)
        {
            return ranges[0];
        }
        else if (i >= ranges.length - 1)
        {
            return ranges[ranges.length - 1];
        }

        double d = distance - ranges[i].distance;
        double speed = lerp(ranges[i].speed, ranges[i+1].speed, d);
        double elevation = lerp(ranges[i].elevation, ranges[i+1].elevation, d);
        return new Range(distance, speed, elevation);
    }

    public Range getRangeStep(double distance)
    {
        Range range = null;
        for (int i = 0; i < ranges.length; ++i)
        {
          if (distance <= ranges[i].distance)
          {
            range = ranges[i];
            break;
          }
        }
        if (range == null && ranges.length > 0)
        {
            range = ranges[ranges.length - 1];
        }
        
        return range;
    }
}
