package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TraySubsystem;

public class RawIndexerCommand extends Command {
    private final IndexerSubsystem indexerSubsystem;
    private final DoubleSupplier speed;

    public RawIndexerCommand(IndexerSubsystem intake, DoubleSupplier speed)
    {
        indexerSubsystem = intake;
        this.speed = speed;

        addRequirements(indexerSubsystem);
    }

    @Override
    public void initialize() {
        //traySubsystem.reset();
    }
  
    @Override
    public void execute() {
        indexerSubsystem.setSpeed(speed.getAsDouble());
    }
  
    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.setSpeed(0);
    }
  
    @Override
    public boolean isFinished() {
        return false;
    }
}
