package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax motor;
  private final SparkMaxConfig motorConfig;
  private final SparkLimitSwitch forwardLimitSwitch;
  private final SparkLimitSwitch reverseLimitSwitch;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController closedLoop;
  private final ClosedLoopConfig pidConfig;

  private double kP = ElevatorConstants.kP;
  private double kI = ElevatorConstants.kI;
  private double kD = ElevatorConstants.kD;

  private double targetPosition = 0.0;

  public ElevatorSubsystem() {
    motor = new SparkMax(ElevatorConstants.LeaderId, MotorType.kBrushless);
    forwardLimitSwitch = motor.getForwardLimitSwitch();
    reverseLimitSwitch = motor.getReverseLimitSwitch();
    encoder = motor.getEncoder();
    closedLoop = motor.getClosedLoopController();

    // Motor config
    motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(IdleMode.kBrake);

    // Limit switches
    motorConfig.limitSwitch
      .forwardLimitSwitchType(Type.kNormallyOpen)
      .forwardLimitSwitchEnabled(true)
      .reverseLimitSwitchType(Type.kNormallyOpen)
      .reverseLimitSwitchEnabled(true);

    // Soft limits
    motorConfig.softLimit
      .forwardSoftLimit(500)
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimit(-500)
      .reverseSoftLimitEnabled(true);

    // PID
    pidConfig = motorConfig.closedLoop;
    pidConfig.p(kP);
    pidConfig.i(kI);
    pidConfig.d(kD);
    pidConfig.outputRange(-ElevatorConstants.OutputLimit, ElevatorConstants.OutputLimit);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    encoder.setPosition(0);

    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);
  }

  @Override
  public void periodic() {
    // Live PID tuning
    double newP = SmartDashboard.getNumber("kP", kP);
    double newI = SmartDashboard.getNumber("kI", kI);
    double newD = SmartDashboard.getNumber("kD", kD);

    if (newP != kP || newI != kI || newD != kD) {
      kP = newP;
      kI = newI;
      kD = newD;
      pidConfig.p(kP);
      pidConfig.i(kI);
      pidConfig.d(kD);
      motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    SmartDashboard.putNumber("Elevator Position", encoder.getPosition());
    SmartDashboard.putNumber("Target Position", targetPosition);
    SmartDashboard.putNumber("Motor Output", motor.getAppliedOutput());
  }

  // ---- Control methods ----
  public void jogUp() {
    motor.set(ElevatorConstants.JogStep);
  }

  public void jogDown() {
    motor.set(-ElevatorConstants.JogStep);
  }

  public void stop() {
    motor.set(0.0);
  }

  public void moveToLevel2() {
    targetPosition = ElevatorConstants.Level2pos;
    closedLoop.setReference(targetPosition, SparkMax.ControlType.kPosition);
  }

  public void moveToLevel1() {
    targetPosition = ElevatorConstants.Level1pos;
    closedLoop.setReference(targetPosition, SparkMax.ControlType.kPosition);
  }

  public void moveToBottom() {
    targetPosition = ElevatorConstants.BottomPos;
    closedLoop.setReference(targetPosition, SparkMax.ControlType.kPosition);
  }
}
