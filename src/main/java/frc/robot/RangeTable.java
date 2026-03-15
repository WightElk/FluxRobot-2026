package frc.robot;

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
//2 - -3.45
//1.75 - 4.7
//1.5 - -5.7
//1.25 - -1.2
//1 - -1.5
//0.75 - -2.6
//0.5
//0.25
//0
// Elevation
// 4.25 in = 20 => k = 4.7
    private Range[] ranges =
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

    public Range getRange(double distance)
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
        if (range == null)
        {
            range = ranges[ranges.length - 1];
        }
        
        return range;
    }
}
