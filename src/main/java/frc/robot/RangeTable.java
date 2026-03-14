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

    private Range[] ranges =
    {
        new Range(17.5, 5000, 2), //0 -1
        new Range(47, 5000, 2), //1.5
        new Range(77, 5000, 2), //2.75
        new Range(107, 5400, 2), //
        new Range(127, 6350, 2),
        new Range(147, 6600, 2),
        new Range(174, 7000, 2) //
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
