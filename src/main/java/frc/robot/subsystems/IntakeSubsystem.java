package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.SparkClosedLoopController;
//import com.revrobotics.spark.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants;
import frc.robot.PIDCtrl;

//import frc.robot.PIDCtrl;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX motor;
    private final TalonFX follower;

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
    /* Start at velocity 0, use slot 1 */
    private final VelocityTorqueCurrentFOC velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(1);
    /* Keep a neutral out so we can disable the motor */
    private final NeutralOut brake = new NeutralOut();

    // private SparkMax rollerMotor;
    // private SparkMax rollerMotorR;

    private RelativeEncoder m_encoder;
    private RelativeEncoder rollerEncoder;

    RelativeEncoder m_alternateEncoder;
    SparkClosedLoopController closedLoopCtrl;
    //private SparkPIDController m_pidController;
    private PIDController m_pidController;

    // PID coefficients
    public double kP = 0.05;//0.1;
    public double kI = 0.0;//1e-4;
    public double kD = 0.0;//1; 
    public double kIz = 0;
    public double kFF= 0;
    public double kMaxOutput= 1;
    public double kMinOutpu = -1;
//    private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
    // private SparkMaxConfig motorConfig;
    // private SparkLimitSwitch forwardLimitSwitch;
    // private SparkLimitSwitch reverseLimitSwitch;
    // private RelativeEncoder encoder;

      private final DoublePublisher  rawDriveVelocityPublisher;

    private final PIDCtrl pidCtrl;
    private double timeDelta;
    private double controlValue;
    /**
     * This subsytem that controls the roller.
     */
    public IntakeSubsystem(int deviceId, CANBus canBus) {
        timeDelta = 0.02;
        controlValue = 0;

        motor = new TalonFX(deviceId, canBus);
        int followerId = IntakeConstants.FollowerId;
        follower = followerId > 0 ? new TalonFX(followerId, canBus) : null;

        TalonFXConfiguration configs = new TalonFXConfiguration();

        /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
        configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
        configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
        configs.Slot0.kI = 0; // No output for integrated error
        configs.Slot0.kD = 0; // No output for error derivative
        // Peak output of 8 volts
        configs.Voltage.withPeakForwardVoltage(Volts.of(8))
        .withPeakReverseVoltage(Volts.of(-8));

        /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
        configs.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
        configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 A output
        configs.Slot1.kI = 0; // No output for integrated error
        configs.Slot1.kD = 0; // No output for error derivative
        // Peak output of 40 A
        configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40)).withPeakReverseTorqueCurrent(Amps.of(-40));

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(configs);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
        if (follower != null)
            follower.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Opposed));

        rawDriveVelocityPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic(
            "Raw Shooter Velocity").publish();

    // Set up the roller motor as a brushed motor
    // rollerMotor = new SparkMax(IntakeConstants.MotorId, MotorType.kBrushless);
    // rollerMotorR = new SparkMax(IntakeConstants.FollowerId, MotorType.kBrushless);
    // m_encoder = rollerMotor.getEncoder();

    //    upShooterMotor = new SparkMax(ShooterConstants.UP_MOTOR_ID, MotorType.kBrushless);

    // upShooterMotorR = new SparkMax(ShooterConstants.UP_MOTOR_IDR, MotorType.kBrushless);
    // downShooterMotor = new SparkMax(ShooterConstants.DOWN_MOTOR_ID, MotorType.kBrushless);
    // downShooterMotorR = new SparkMax(ShooterConstants.DOWN_MOTOR_IDR, MotorType.kBrushless);

    //downShooterMotor = new SparkMax(ShooterConstants.DOWN_MOTOR_ID, MotorType.kBrushless);

    
    //m_alternateEncoder = upShooterMotor.getAlternateEncoder();
//    m_pidController = upShooterMotor.getPIDController();
//    m_pidController.setFeedbackDevice(m_alternateEncoder);

//    rollerEncoder = rollerMotor.getEncoder();

    // forwardLimitSwitch = upShooterMotor.getForwardLimitSwitch();
    // reverseLimitSwitch = upShooterMotor.getReverseLimitSwitch();
    
    // closedLoopCtrl = upShooterMotor.getClosedLoopController();

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    // rollerMotor.setCANTimeout(250);
    // rollerMotorR.setCANTimeout(250);
//    upShooterMotor.setCANTimeout(250);
    //downShooterMotor.setCANTimeout(250);

    // Create and apply configuration for roller motor. Voltage compensation helps
    // the roller behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the roller stalls.
//    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    // rollerConfig.voltageCompensation(IntakeConstants.ROLLER_MOTOR_VOLTAGE_COMP);
    // rollerConfig.smartCurrentLimit(IntakeConstants.ROLLER_MOTOR_CURRENT_LIMIT);

//    rollerConfig.closedLoop.pid(kP, kI, kD);
/*
    // Enable limit switches to stop the motor when they are closed
    rollerConfig.limitSwitch
        .forwardLimitSwitchType(Type.kNormallyOpen)
        .forwardLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);

    // Set the soft limits to stop the motor at -50 and 50 rotations
    rollerConfig.softLimit
        .forwardSoftLimit(50)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(-50)
        .reverseSoftLimitEnabled(true);
*/     
    //com.revrobotics.spark.SparkLimitSwitch forwLimit = upShooterMotor.getForwardLimitSwitch();

//    upShooterMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // rollerConfig.idleMode(IdleMode.kBrake);
    // rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

 //TODO   
    // rollerConfig.follow(IntakeConstants.LeaderId, true);
    // rollerMotorR.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // m_pidController = new PIDController(kP, kI, kD);
    // //m_pidController.setIZone(kIz);
    // m_pidController.setSetpoint(1);

    // set PID coefficients
    // m_pidController.setIZone(kIz);
    // m_pidController.setFF(kFF);
    // m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        pidCtrl = new PIDCtrl(kP, kI, kD, timeDelta);

//        SmartDashboard.putNumber("Coeff", 0.15);
    }

    @Override
    public void periodic() {
        //motor.getPosition(), motor.getVelocity());
    }

    public void init() {
        m_encoder.setPosition(0);
        // m_pidController.reset();
        // m_pidController.setSetpoint(10);
        pidCtrl.reset();
        controlValue = 0;
    }

    public void setSpeed(double speed) {
        motor.setControl(velocityVoltage.withVelocity(speed * Constants.IntakeConstants.MaxMotorRPS));
//        motor.setControl(velocityTorque.withVelocity(speed * Constants.MaxMotorRPS));
    }

    public void stop() {
        // Disable the motor
        motor.setControl(brake);
    }

    /**
     *  This is a method that makes the roller spin to your desired speed.
     *  Positive values make it spin forward and negative values spin it in reverse.
     * 
     * @param speedmotor speed from -1.0 to 1, with 0 stopping it
     */
    public void runRoller(double rotationsPerSecond){
//        rollerMotor.set(speed);
//        System.out.println("Roller: " + rollerMotor.getAppliedOutput() + " - " + rollerMotorR.getAppliedOutput());
  
  //      double rotationsPerSecond = joyValue * 50; // Go for plus/minus 50 rotations per second

        /* Use velocity voltage */
        motor.setControl(velocityVoltage.withVelocity(rotationsPerSecond));
        /* Use velocity torque */
//        m_fx.setControl(m_velocityTorque.withVelocity(desiredRotationsPerSecond));
    }

    // public void runShooter(double speed) {
    //     upShooterMotor.set(speed);
    // }
/*
    public void runShooter1(double speed) {
        double setPoint = 50;

        double ctrlSpeed = m_pidController.calculate(m_encoder.getPosition(), setPoint);
//        m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
        System.out.println("Pos/Vel: " + m_encoder.getPosition() + " - " + m_encoder.getVelocity());
        System.out.println("PID1: " + ctrlSpeed);
//            SmartDashboard.putNumber("Applied Output", upShooterMotor.getAppliedOutput());
            //System.out.println("PID: " + m_pidController.calculate(m_encoder.getVelocity()));
            //System.out.println("Encoder A: " + m_alternateEncoder.getPosition() + "-" + m_alternateEncoder.getVelocity());
        //upShooterMotor.set(m_pidController.calculate(m_encoder.getPosition()));
//        double coeff = SmartDashboard.getNumber("Coeff", 1);
        //ctrlSpeed *= coeff;
//        System.out.println("Speed: " + speed);

        if (controlValue == 0)
            controlValue = speed;
        double dif = pidCtrl.calculateDif(m_encoder.getPosition(), setPoint);
        controlValue += dif;
        System.out.println("PID Dif/Control: " + dif + " - " + controlValue);

        double limited = PIDCtrl.limitRange(controlValue, 1.0);

//        closedLoopCtrl.setReference(1000, ControlType.kVelocity);
//        closedLoopCtrl.setReference(50, ControlType.kPosition);

//        upShooterMotor.setVoltage(ctrlSpeed);
        upShooterMotor.set(controlValue);
//        upShooterMotor.setVoltage(controller.update(getState().getTargetAngle().getDegrees(), getCurrentAngle().getDegrees()));

        System.out.println("Applied Output: " + upShooterMotor.getAppliedOutput());            
    }
*/
    public void putParams() {
        // SmartDashboard.putNumber("Linear Sensitivity", m_linCoef);
      }
  
    public void getParams() {
//      m_linCoef = SmartDashboard.getNumber("Linear Sensitivity", LinCoef);

    //   if (m_cuspX > 0.9)
    //     m_cuspX = 0.9;
    }

  public void updateTelemetry()
  {
    // if (absoluteEncoder != null)
    // {
    //   rawAbsoluteAnglePublisher.set(absoluteEncoder.getAbsolutePosition());
    // }
    {
    //   rawAnglePublisher.set(angleMotor.getPosition());
    //   rawDriveEncoderPublisher.set(drivePositionCache.getValue());
    //     double velocity = getVelocity();
    //   rawDriveVelocityPublisher.set(driveVelocityCache.getValue());
    }
    // adjAbsoluteAnglePublisher.set(getAbsolutePosition());
    // absoluteEncoderIssuePublisher.set(getAbsoluteEncoderReadIssue());
  }
}