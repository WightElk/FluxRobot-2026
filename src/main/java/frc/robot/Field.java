package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Field {
    public double length = Units.inchesToMeters(357);
    public double width = Units.inchesToMeters(317.375);
    public Translation2d hubPos = new Translation2d();

    public Field()
    {
        
    }

    public Translation2d getHub()
    {
        return hubPos;
    } 
}
