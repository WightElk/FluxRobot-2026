package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TraySubsystem;

public class RunShooterCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final DoubleSupplier speed;

    public RunShooterCommand(ShooterSubsystem shooter, DoubleSupplier speed)
    {
        shooterSubsystem = shooter;
        this.speed = speed;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        //traySubsystem.reset();
    }
  
    @Override
    public void execute() {
        shooterSubsystem.setSpeed(speed.getAsDouble());
    }
  
    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setSpeed(0);
    }
  
    @Override
    public boolean isFinished() {
        return false;
    }
}
