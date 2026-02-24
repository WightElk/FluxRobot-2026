package frc.robot.autos;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.DriveConstants;
import frc.robot.generated.TunerConstants;

public class DriveForwardAuto extends Command {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private CommandSwerveDrivetrain drivetrain;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private Timer timer = new Timer();
    private double driveTime = DriveConstants.AutoModeDriveTime;//3.25;

      /**
     * This auto will have the robot drive forwards
     * 
     * There are many ways to write autos, this form will work well for most simple
     * auto routines. For more advanced routines you may want a different structure and 
     * to use more sensors.
     * 
     * Here we use a single timer gate, after the robot has finished driving for the first 3.25 
     * seconds it will stop moving. You may wish for the robot to move more or less depending on
     * your use case.
     * 
     * @param drive
     */
    public DriveForwardAuto(CommandSwerveDrivetrain drivetrain)
    {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
  public void initialize() {
    // start timer, uses restart to clear the timer as well in case this command has
    // already been run before
    timer.restart();
  }

  // Runs every cycle while the command is scheduled (~50 times per second), here we will just drive forwards
  @Override
  public void execute() {
    // drive forward at 30% speed
    if(timer.get() < driveTime)
    {
      double speed = drivetrain.allianceColor == Alliance.Red ? DriveConstants.AutoModeSpeed : -DriveConstants.AutoModeSpeed;
      drivetrain.setControl(drive.withVelocityX(speed)
        .withVelocityY(0)
        .withRotationalRate(0));
    }
  }

  // Runs each time the command ends via isFinished or being interrupted.
  @Override
  public void end(boolean isInterrupted) {
    // stop drive motors
    drivetrain.setControl(drive.withVelocityX(0)
      .withVelocityY(0)
      .withRotationalRate(0));

    timer.stop();
  }

  // Runs every cycle while the command is scheduled to check if the command is finished
  @Override
  public boolean isFinished() {
    // check if timer exceeds driving time in seconds, when it has this will return true indicating
    // this command is finished
    return timer.get() >= driveTime;
  }
}
