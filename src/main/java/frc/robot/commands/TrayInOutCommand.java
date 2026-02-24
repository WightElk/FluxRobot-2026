package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.TraySubsystem;

public class TrayInOutCommand extends Command {
    private final TraySubsystem traySubsystem;
    private final DoubleSupplier speed;

    public TrayInOutCommand(TraySubsystem tray, DoubleSupplier speed)
    {
        traySubsystem = tray;
        this.speed = speed;

        addRequirements(tray);
    }

    @Override
    public void initialize() {
        traySubsystem.reset();
    }
  
    @Override
    public void execute() {
        traySubsystem.run();
    }
  
    @Override
    public void end(boolean interrupted) {
        traySubsystem.setSpeed(0);
    }
  
    @Override
    public boolean isFinished() {
        return traySubsystem.isCoralIn() || traySubsystem.isCoralOut();
    }
}
