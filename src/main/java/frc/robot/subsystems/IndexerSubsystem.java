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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.PIDCtrl;

//import frc.robot.PIDCtrl;

public class IndexerSubsystem extends SubsystemBase {

    private final TalonFX motor;
    private final TalonFX follower;
    // private final TalonFX leftFeeder;
    // private final TalonFX rightFeeder;

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage velocityVoltage1 = new VelocityVoltage(0).withSlot(0);
    /* Start at velocity 0, use slot 1 */
    private final VelocityTorqueCurrentFOC velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(1);
    /* Keep a neutral out so we can disable the motor */
    private final NeutralOut brake = new NeutralOut();

    private RelativeEncoder m_encoder;
    private RelativeEncoder rollerEncoder;

    RelativeEncoder m_alternateEncoder;
    SparkClosedLoopController closedLoopCtrl;
    //private SparkPIDController m_pidController;
    private PIDController m_pidController;

    // PID coefficients
    public double kP = 0.1;//0.1;
    public double kI = 0.0;//1e-4;
    public double kD = 0.05;//1; 
    public double kIz = 0;
    public double kFF= 0;
    public double kMaxOutput= 1;
    public double kMinOutpu = -1;

    private final PIDCtrl pidCtrl;
    private double timeDelta;
    private double controlValue;
    /**
     * This subsytem that controls the roller.
     */
    public IndexerSubsystem(CANBus canBus) {
        timeDelta = 0.02;
        controlValue = 0;

        motor = new TalonFX(Constants.IndexerConstants.IndexerId, canBus);
        int followerId = Constants.IndexerConstants.FeederId;
        follower = followerId > 0 ? new TalonFX(followerId, canBus) : null;
        // leftFeeder = new TalonFX(Constants.IndexerConstants.LeftFeederId, canBus);
        // rightFeeder = new TalonFX(Constants.IndexerConstants.RightFeederId, canBus);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        TalonFXConfiguration configs1 = new TalonFXConfiguration();

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

                /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
        configs1.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
        configs1.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        configs1.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
        configs1.Slot0.kI = 0; // No output for integrated error
        configs1.Slot0.kD = 0; // No output for error derivative
        // Peak output of 8 volts
        configs1.Voltage.withPeakForwardVoltage(Volts.of(8))
        .withPeakReverseVoltage(Volts.of(-8));

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(configs);
            // status = leftFeeder.getConfigurator().apply(configs);
            // status = rightFeeder.getConfigurator().apply(configs);
            // if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
        status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = follower.getConfigurator().apply(configs1);
            // status = leftFeeder.getConfigurator().apply(configs);
            // status = rightFeeder.getConfigurator().apply(configs);
            // if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
        // if (follower != null)
        //     follower.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Aligned));

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
        double rps = - speed  * Constants.IndexerConstants.MaxMotorRPS;

        motor.setControl(velocityVoltage.withVelocity(rps));
        follower.setControl(velocityVoltage1.withVelocity(- rps));
        // leftFeeder.setControl(velocityVoltage.withVelocity(speed * Constants.MaxMotorRPS));
        // rightFeeder.setControl(velocityVoltage.withVelocity(- speed * Constants.MaxMotorRPS));

        //motor.setControl(velocityVoltage.withVelocity(speed * Constants.ShooterConstants.MaxMotorRPS));
//        .withFeedForward(feedforward))
//        motor.setControl(velocityTorque.withVelocity(speed * Constants.MaxMotorRPS));
        double v = motor.getVelocity().getValue().magnitude();
        System.out.println("Index: " + speed + " / " + rps + " / " + v);

//        motor.setControl(velocityTorque.withVelocity(speed * Constants.MaxMotorRPS));
    }

    public void stop() {
        // Disable the motor
        motor.setControl(brake);
        follower.setControl(brake);
        // leftFeeder.setControl(brake);
        // rightFeeder.setControl(brake);

    }

    /**
     *  This is a method that makes the roller spin to your desired speed.
     *  Positive values make it spin forward and negative values spin it in reverse.
     * 
     * @param speedmotor speed from -1.0 to 1, with 0 stopping it
     */
    public void runRoller(double speed){
        // motor.set(speed);
        // motorR.set(-speed);
//        System.out.println("Roller: " + motor.getAppliedOutput() + " - " + motorR.getAppliedOutput());
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
}