package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import frc.robot.Constants;
import frc.robot.subsystems.PositionMech;

public class PositionCmd extends Command
{
    private final PositionMech positionMech;
    private final DoubleSupplier position;

    public PositionCmd(PositionMech positionMech, DoubleSupplier position)
    {
        this.positionMech = positionMech;
        this.position = position;

        addRequirements(positionMech);
    }

    @Override
    public void initialize()
    {
        positionMech.reset();
        double p = position.getAsDouble();
        positionMech.setTargetPosition(p);
    }
  
    @Override
    public void execute()
    {
        double p = position.getAsDouble();
        positionMech.run(p);
    }
  
    @Override
    public void end(boolean interrupted)
    {
        //if (interrupted)
        positionMech.stop();
    }
  
    @Override
    public boolean isFinished()
    {
        //return positionMech.atTarget();
        return false;
    }
}
