package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VelocityMech2;
import frc.robot.subsystems.VelocitySubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** A command to take Algae into the robot. */
public class ShootCommand extends Command {
    private final VelocityMech2 velocityMech;
    private final DoubleSupplier speed;
    private final int direction;

    /**
     * Rolls Algae into the intake.
     *
     * @param roller The subsystem used by this command.
     */
    public ShootCommand(VelocityMech2 velocityMech, DoubleSupplier speed, int direction) {
        this.velocityMech = velocityMech;
        this.speed = speed;
        this.direction = direction;    

        double v = speed.getAsDouble();
        // if (direction != Constants.Forward)
        //     v = -v;
        velocityMech.setTargetSpeed(v);
        addRequirements(velocityMech);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        if (velocityMech.running())
        {
            velocityMech.stop();
        }
        else
        {
            velocityMech.reset();

            double v = speed.getAsDouble();
            if (direction != Constants.Forward)
                v = -v;
             //velocityMech.setTargetSpeed(v);
             //Do Run
             velocityMech.setSpeed(v);
            //velocityMech.setSpeed(direction);
        }
    }

    @Override
    public void execute()
    {
        // double v = speed.getAsDouble();
        // if (direction != Constants.Forward)
        //     v = -v;
        // velocityMech.setSpeed(v);
    }
  
    @Override
    public void end(boolean interrupted)
    {
        velocityMech.stop();
    }
  
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
