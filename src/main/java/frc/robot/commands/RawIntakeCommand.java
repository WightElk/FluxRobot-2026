package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TraySubsystem;

public class RawIntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final DoubleSupplier speed;

    public RawIntakeCommand(IntakeSubsystem intake, DoubleSupplier speed)
    {
        intakeSubsystem = intake;
        this.speed = speed;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        //traySubsystem.reset();
    }
  
    @Override
    public void execute() {
        intakeSubsystem.setSpeed(speed.getAsDouble());
    }
  
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setSpeed(0);
    }
  
    @Override
    public boolean isFinished() {
        return false;
    }
}
