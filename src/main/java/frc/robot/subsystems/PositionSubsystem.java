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
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants;
import frc.robot.PIDCtrl;

public class PositionSubsystem extends SubsystemBase {
    private final String name;
    private final TalonFX motor;
    private final TalonFX follower;
    private final TalonFXConfiguration configs;

    private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
    /* Start at velocity 0, use slot 1 */
    private final PositionTorqueCurrentFOC positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);
    /* Keep a neutral out so we can disable the motor */
    private final NeutralOut brake = new NeutralOut();

    // private final RelativeEncoder upEncoder;

    private double targetPosition = DefaultPosition;
    private double position = 0;
    private boolean running = false;
    private boolean atTarget = false;
    private boolean targetPositionChanged = false;
    private double velocityRPM = 0;
    private double velocity = 0;

    private final PIDCtrl pidCtrl;
    private final PIDController pidController;

    private double timeDelta = Constants.TimePeriod;

    public static final double DefaultPosition = 100.0;
    public static final double MaxMotorRPM = 6000;

        // configs.Slot0.kP = kS;//2.4; 
        // configs.Slot0.kI = kV;//0.0;
        // configs.Slot0.kD = kP;//0.1; 
    public static final double DefaultKP = 2.4;  // An error of 1 rotation results in 2.4 V output
    public static final double DefaultKD = 0.1;  // A velocity of 1 rps results in 0.1 V output
    public static final double DefaultKI = 0.0; // No output for integrated error
    public static final double DefaultKV = 0.0; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    public static final double DefaultKS = 0.0; // To account for friction, add 0.1 V of static feedforward
    public static final double DefaultMaxOutput = 1.0;
    public static final double DefaultMinOutput = -1.0;
    public static final double DeltaPos = 50;

    public static final int JogStep = -1;

    // PID coefficients
    public double kP = DefaultKP;
    public double kD = DefaultKD;
    public double kI = DefaultKI;
    public double kV = DefaultKV;
    public double kS = DefaultKS;
    // public double kIz = 0;
    // public double kFF= 0;
    public double kMaxOutput = DefaultMaxOutput;
    public double kMinOutput = DefaultMinOutput;

    public double posDelta = DeltaPos;
    private int jogStep = JogStep;


    private int execCounter = 0;
    private double time = 0;
    private double controlValueUp = 0;
    private double controlValueDown = 0;
    private boolean atPosition = false;

    /**
     * This subsytem that controls the roller.
     */
    public PositionSubsystem(CANBus canBus, String name, int motorId, int followerId) {
        this.name = name;
        motor = new TalonFX(motorId, canBus);
        follower = followerId > 0 ? new TalonFX(followerId, canBus) : null;

        configs = new TalonFXConfiguration();

        setConfig();

        pidController = new PIDController(kP, kI, kD);
        //m_pidController.setIZone(kIz);
        //m_pidController.setSetpoint(1);

    // set PID coefficients
    // m_pidController.setIZone(kIz);
    // m_pidController.setFF(kFF);
    // m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        pidCtrl = new PIDCtrl(kP, kD, kI, timeDelta);
    }

    @Override
    public void periodic() {
        if (running && targetPositionChanged)
        {
            double pos = targetPosition;
            motor.setControl(positionVoltage.withPosition(pos));
            if (follower != null)
                follower.setControl(positionVoltage.withPosition(pos));
            System.out.println("setControl-periodic");
        }

        double pos = getPosition();
        if (pos != position)
        {
            String prefix = name + "/";
            SmartDashboard.putNumber(prefix + "Pos", pos);
            position = pos;
        }
        atPosition = Math.abs(pos - targetPosition) <= posDelta;
        //System.out.println("Pos: " + position + " / " + pos);
    }

    public boolean atSetPoint() {
        return atPosition;
    }

    public void init(double rpm) {
        System.out.println(name + " Initializing");
        getParams();

        execCounter = 0;
        time = 0;
        controlValueUp = 0;
        controlValueDown = 0;
        running = false;
        targetPositionChanged = false;
        atTarget = false;

        // upEncoder.setPosition(0);

        // upPidCtrl.reset();
        // m_pidController.reset();
        // m_pidController.setSetpoint(10);
    }

    public void reset()
    {
        System.out.println(name + " Reset");

        execCounter = 0;
        time = 0;
        controlValueUp = 0;
        controlValueDown = 0;
        running = false;
        targetPositionChanged = false;
        atTarget = false;
    }

    public void jogUp()
    {
        double pos = getPosition();
        pos += jogStep;
        targetPosition = pos;
        motor.setControl(positionVoltage.withPosition(pos));
    }
    
    public void jogDown()
    {
        double pos = getPosition();
        pos -= jogStep;
        targetPosition = pos;
        motor.setControl(positionVoltage.withPosition(pos));
    }
    
    public void run() {
        // speed = -speed;
        // double rps = speed  * Constants.ShooterConstants.MaxMotorRPS;
        if (targetPosition != position)
        {
            position = targetPosition;
            motor.setControl(positionVoltage.withPosition(- position));
            System.out.println("setControl");
        }
//        .withFeedForward(feedforward))
        // double v = motor.getVelocity().getValue().magnitude();
        // System.out.println("Speed: " + velocityRPM + " / " + v);
    }

    public void setPosition(double pos) {
//        pos = -pos;
        double rps = pos  * Constants.ShooterConstants.MaxMotorRPS;

        motor.setControl(positionVoltage.withPosition(pos));
//        .withFeedForward(feedforward))
        if (follower != null)
            follower.setControl(positionVoltage.withPosition(pos));

        double p = motor.getPosition().getValue().in(Rotations);
        System.out.println("Position: " + pos + " / " + targetPosition  + " / " + p);
    }

    public void stop() {
        motor.setControl(brake);
        if (follower != null)
            follower.setControl(brake);
        reset();
    }

    public double getPosition()
    {
        return motor.getPosition().getValue().in(Rotations);
    }

    public void runDown(double target) {
        // double velUp = upEncoder    .getVelocity();
        // double velDown = downEncoder.getVelocity();
//        upShooterMotor.set(target);
//        downShooterMotor.set(-target);
        // SmartDashboard.putNumber("Shooter RPM Up", velUp);
        // SmartDashboard.putNumber("Shooter RPM Down", velDown);
    }

    public void setConfig()
    {
        /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
        // configs.Slot0.kS = kS;//0.         01; // To account for friction, add 0.1 V of static feedforward
        // configs.Slot0.kV = kV;//0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        configs.Slot0.kP = kP;//0.11; // An error of 1 rotation per second results in 0.11 V output
        configs.Slot0.kI = kI; // No output for integrated error
        configs.Slot0.kD = kD; // No output for error derivative
        // Peak output of 8 volts
        configs.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));

        /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
        // configs.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward

        // configs.Slot1.kP = 60; // An error of 1 rotation per second results in 5 A output
        // configs.Slot1.kI = 0; // No output for integrated error
        // configs.Slot1.kD = 6; // No output for error derivative
        // // Peak output of 40 A
        // configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40)).withPeakReverseTorqueCurrent(Amps.of(-40));

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(configs);
            if (status.isOK())
                break;
            // if (follower != null)
            //     status = follower.getConfigurator().apply(configs);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
        if (follower != null)
            follower.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Opposed));

        motor.setPosition(0);

    }

    public void putParams() {
        String prefix = name + "/";

        SmartDashboard.setPersistent(prefix + "Pos");
        SmartDashboard.setPersistent(prefix + "kP");
        SmartDashboard.setPersistent(prefix + "kD");
        SmartDashboard.setPersistent(prefix + "kI");
        SmartDashboard.setPersistent(prefix + "kV");
        SmartDashboard.setPersistent(prefix + "kS");
        SmartDashboard.setPersistent(prefix + "MaxOutput");
        SmartDashboard.setPersistent(prefix + "MinOutput");
        SmartDashboard.setPersistent(prefix + "PosDelta");

        SmartDashboard.putNumber(prefix + "Target Pos", targetPosition);
        SmartDashboard.putNumber(prefix + "Pos", position);

        SmartDashboard.putNumber(prefix + "kP", kP);
        SmartDashboard.putNumber(prefix + "kD", kD);
        SmartDashboard.putNumber(prefix + "kI", kI);
        SmartDashboard.putNumber(prefix + "kV", kV);
        SmartDashboard.putNumber(prefix + "kS", kS);
        SmartDashboard.putNumber(prefix + "MaxOutput", kMaxOutput);
        SmartDashboard.putNumber(prefix + "MinOutput", kMinOutput);

        SmartDashboard.putNumber(prefix + "PosDelta", posDelta);
    }

    public void getParams() {
        String prefix = name + "/";

        kP = SmartDashboard.getNumber(prefix + "kP", DefaultKP);
        kD = SmartDashboard.getNumber(prefix + "kD", DefaultKD);
        kI = SmartDashboard.getNumber(prefix + "kI", DefaultKI);
        kV = SmartDashboard.getNumber(prefix + "kV", DefaultKV);
        kS = SmartDashboard.getNumber(prefix + "kS", DefaultKS);
        kMaxOutput = SmartDashboard.getNumber(prefix + "MaxOutput", kMaxOutput);
        kMinOutput = SmartDashboard.getNumber(prefix + "MinOutput", kMinOutput);
        posDelta = SmartDashboard.getNumber(prefix + "posDelta", DeltaPos);

        setConfig();

        double pos = SmartDashboard.getNumber(prefix + "Target Pos", DefaultPosition);
        if (pos != targetPosition)
        {
            targetPosition = pos;
            targetPositionChanged = true;
        }

        pidCtrl.pid(kP, kD, kI);
    }
}
