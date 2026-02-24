package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.S2FloatStateValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.TrayConstants;

public class TraySubsystem extends SubsystemBase {
  enum Stage {
    InStage,
    OutStage
  }

  private final SparkMax motor;
  private final SparkMax feedMotor;
  private final CANdi switches;

  private double speed = TrayConstants.Speed;
  private double inSpeed = TrayConstants.InSpeed;
  private double outSpeed = TrayConstants.OutSpeed;

  private boolean inside = false;
  private boolean outside = false;
  private boolean prevActivated1 = false;
  private boolean prevActivated2 = false;
  private Stage stage = Stage.InStage;

  public TraySubsystem(CANBus canBus) {
    switches = new CANdi(TrayConstants.CanDiId, canBus);
    motor = new SparkMax(TrayConstants.MotorId, MotorType.kBrushless);
    feedMotor = new SparkMax(TrayConstants.FeedMotorId, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config.follow(TrayConstants.MotorId, true);
    feedMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    // Configure CANdi
    CANdiConfiguration switchesConfig = new CANdiConfiguration();
    switchesConfig.DigitalInputs.S1FloatState = S1FloatStateValue.FloatDetect;
    switchesConfig.DigitalInputs.S2FloatState = S2FloatStateValue.FloatDetect;
    // Switches will be closeed to Ground
    switchesConfig.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenLow;
    switchesConfig.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenLow;

    switches.getConfigurator().apply(switchesConfig);

    // Add a SmartDashboard slider for manual control
    SmartDashboard.putNumber("Coral Motor Speed", speed);
    SmartDashboard.putNumber("Coral In Speed", inSpeed);
    SmartDashboard.putNumber("Coral Out Speed", outSpeed);
  }

  public void reset() {
    inside = outside = false;
    prevActivated1 = false;
    prevActivated2 = false;
    stage = stage == Stage.OutStage ? Stage.InStage : Stage.OutStage;

    boolean activated2 = !switches.getS2Closed().getValue();
    prevActivated2 = activated2;
    stage = activated2 ? Stage.OutStage : Stage.InStage;

    speed = SmartDashboard.getNumber("Coral Motor Speed", TrayConstants.Speed);
    inSpeed = SmartDashboard.getNumber("Coral In Speed", TrayConstants.InSpeed);
    outSpeed = SmartDashboard.getNumber("Coral Out Speed", TrayConstants.OutSpeed);
  }

  public boolean isCoralIn() {
    return inside;
  }

  public boolean isCoralOut() {
    return outside;
  }

  public void setSpeed(double speed) {
    motor.set(speed);
    SmartDashboard.putNumber("Coral Motor Applied Output", motor.getAppliedOutput());
  }

  public void run() {
    boolean activated1 = !switches.getS1Closed().getValue();
    boolean activated2 = !switches.getS2Closed().getValue();

    if (stage == Stage.OutStage) {
      outside = !activated2 && prevActivated2;

      motor.set(-outSpeed);
    }
    else {
      inside = !activated1 && prevActivated1;
      // if (inside)
      //   stage = Stage.InStage;

      double speed = !activated1 ? inSpeed : 0.5 * inSpeed;
      motor.set(-speed);
    }

    prevActivated1 = activated1;
    prevActivated2 = activated2;

    SmartDashboard.putNumber("Coral Motor Applied Output", motor.getAppliedOutput());
  }
}
