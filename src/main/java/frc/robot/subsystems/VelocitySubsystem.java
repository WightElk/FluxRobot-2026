package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import org.littletonrobotics.junction.AutoLogOutput;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants;
import frc.robot.PIDCtrl;

public class VelocitySubsystem extends SubsystemBase {
    private final String name;
    private final TalonFX motor;
    private final TalonFX follower;
    private final TalonFXConfiguration configs;
    private final TalonFXConfiguration configs1;

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage velocityVoltage1 = new VelocityVoltage(0).withSlot(0);
    /* Start at velocity 0, use slot 1 */
    private final VelocityTorqueCurrentFOC velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(1);
    /* Keep a neutral out so we can disable the motor */
    private final NeutralOut brake = new NeutralOut();

    // private final RelativeEncoder upEncoder;

    private int direction = Constants.Backward;
    private boolean running = false;
    private boolean targetVelocityChanged = false;
    private double velocityRPM = 0;
    private double targetVelocity = DefaultVelocityRPM;
    @AutoLogOutput(key = "{name}/Velocity")
    private double velocity = 0;

    private final PIDCtrl pidCtrl;
    private final PIDController pidController;

    private double timeDelta = Constants.TimePeriod;

    public static final double DefaultVelocityRPM = 3000.0;
    public static final double MaxMotorRPM = 6000;

        // configs.Slot0.kS = kS;//0.01; 
        // configs.Slot0.kV = kV;//0.12;
        // configs.Slot0.kP = kP;//0.11; 
    public static final double DefaultKP = 0.11;  // An error of 1 rotation per second results in 0.11 V output
    public static final double DefaultKD = 0.0;
    public static final double DefaultKI = 0.0;
    public static final double DefaultKV = 0.0; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    public static final double DefaultKS = 0.0; // To account for friction, add 0.1 V of static feedforward
    public static final double DefaultMaxOutput = 1.0;
    public static final double DefaultMinOutput = -1.0;
    public static final double DeltaRPM = 100;

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

    public double rpmDelta = DeltaRPM;


    private int execCounter = 0;
    private double time = 0;
    private double controlValueUp = 0;
    private double controlValueDown = 0;
    private boolean atSpeed = false;

    /**
     * This subsytem that controls the roller.
     */
    public VelocitySubsystem(CANBus canBus, String name, int motorId, int followerId) {
        this.name = name;
        motor = new TalonFX(motorId, canBus);
        follower = followerId > 0 ? new TalonFX(followerId, canBus) : null;

        configs = new TalonFXConfiguration();
        configs1 = new TalonFXConfiguration();

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
        if (running && targetVelocityChanged)
        {
            velocityRPM = direction == Constants.Backward ? -targetVelocity : targetVelocity;
            motor.setControl(velocityVoltage.withVelocity(velocityRPM / 60.0));
            if (follower != null)
                follower.setControl(velocityVoltage.withVelocity(- velocityRPM / 60.0));
            System.out.println("setControl-periodic");
        }

        double vel = 60 * getVelocity();
        if (vel != velocity)
        {
            String prefix = name + "/";
            SmartDashboard.putNumber(prefix + "RPM", vel);
            velocity = vel;
        }
        double target = direction == Constants.Backward ? -targetVelocity : targetVelocity;
        atSpeed = Math.abs(vel - target) <= rpmDelta;
        //System.out.println("RPM: " + velocityRPM + " / " + vel);
    }

    public boolean atSetPoint() {
        return atSpeed;
    }

    public void init(double rpm) {
        System.out.println(name + " Initializing");
        getParams();

        velocityRPM = rpm;
        execCounter = 0;
        
        time = 0;
        controlValueUp = 0;
        controlValueDown = 0;
        running = false;
        targetVelocityChanged = false;
        atSpeed = false;

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
        targetVelocityChanged = false;
        atSpeed = false;
        velocityRPM = 0;
    }

    public void run() {
        run(Constants.Forward);
    }

    public void run(int direction) {
        this.direction = direction;
        // speed = -speed;
        // double rps = speed  * Constants.ShooterConstants.MaxMotorRPS;
        double setRpm = direction == Constants.Backward ? -targetVelocity : targetVelocity;
        if (setRpm != velocityRPM)
        {
            velocityRPM = setRpm;
            motor.setControl(velocityVoltage.withVelocity(velocityRPM / 60.0));
            if (follower != null)
                follower.setControl(velocityVoltage.withVelocity(- velocityRPM / 60.0));
            System.out.println("setControl");
        }
//        .withFeedForward(feedforward))
        running = true;
        double v = motor.getVelocity().getValue().magnitude();
        System.out.println("Run: " + velocityRPM + " / " + v);
    }

    public void setSpeed(double speed) {
        targetVelocityChanged = true;
        running = true;
        speed = -targetVelocity;

        //speed = -speed;
        double rps = speed;//  * Constants.ShooterConstants.MaxMotorRPS;

        //motor.setControl(velocityVoltage.withVelocity(speed));
//        .withFeedForward(feedforward))
//        motor.setControl(velocityTorque.withVelocity(speed * Constants.MaxMotorRPS));
        if (follower != null)
            follower.setControl(velocityVoltage.withVelocity(speed));
        double v = motor.getVelocity().getValue().magnitude();
        System.out.println("SetSpeed: " + speed + " / " + rps + " / " + v);
    }

    public void stop() {
        motor.setControl(brake);
        if (follower != null)
            follower.setControl(brake);
        reset();
    }

    public double getVelocity()
    {
        return motor.getVelocity().getValue().magnitude();
    }

    public void runDown(double target) {
        // double velUp = upEncoder.getVelocity();
        // double velDown = downEncoder.getVelocity();
//        upShooterMotor.set(target);
//        downShooterMotor.set(-target);
        // SmartDashboard.putNumber("Shooter RPM Up", velUp);
        // SmartDashboard.putNumber("Shooter RPM Down", velDown);
    }

    public void setConfig()
    {
        /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
        configs.Slot0.kS = kS;//0.01; // To account for friction, add 0.1 V of static feedforward
        configs.Slot0.kV = kV;//0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        configs.Slot0.kP = kP;//0.11; // An error of 1 rotation per second results in 0.11 V output
        configs.Slot0.kI = kI; // No output for integrated error
        configs.Slot0.kD = kD; // No output for error derivative
        // Peak output of 8 volts
        configs.Voltage.withPeakForwardVoltage(Volts.of(12)).withPeakReverseVoltage(Volts.of(-12));

        /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
        // configs.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
        // configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 A output
        // configs.Slot1.kI = 0; // No output for integrated error
        // configs.Slot1.kD = 0; // No output for error derivative
        // // Peak output of 40 A
        // configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40)).withPeakReverseTorqueCurrent(Amps.of(-40));

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(configs);
            if (status.isOK())
                break;
            //if (follower != null)
              //  status = follower.getConfigurator().apply(configs);
            //if (status.isOK())
              //  break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

        configs1.Slot0.kS = kS;//0.01; // To account for friction, add 0.1 V of static feedforward
        configs1.Slot0.kV = kV;//0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        configs1.Slot0.kP = kP;//0.11; // An error of 1 rotation per second results in 0.11 V output
        configs1.Slot0.kI = kI; // No output for integrated error
        configs1.Slot0.kD = kD; // No output for error derivative
        // Peak output of 8 volts
        configs1.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));

        status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            if (follower != null)
                status = follower.getConfigurator().apply(configs1);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

        // if (follower != null)
        //     follower.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void putParams() {
        String prefix = name + "/";

        SmartDashboard.putNumber(prefix + "Set RPM", targetVelocity);
        SmartDashboard.putNumber(prefix + "RPM", velocity);

        SmartDashboard.putNumber(prefix + "kP", kP);
        SmartDashboard.putNumber(prefix + "kD", kD);
        SmartDashboard.putNumber(prefix + "kI", kI);
        SmartDashboard.putNumber(prefix + "kV", kV);
        SmartDashboard.putNumber(prefix + "kS", kS);
        SmartDashboard.putNumber(prefix + "MaxOutput", kMaxOutput);
        SmartDashboard.putNumber(prefix + "MinOutput", kMinOutput);

        SmartDashboard.putNumber(prefix + "RpmDelta", rpmDelta);

        SmartDashboard.setPersistent(prefix + "Set RPM");
        SmartDashboard.setPersistent(prefix + "RPM");
        SmartDashboard.setPersistent(prefix + "kP");
        SmartDashboard.setPersistent(prefix + "kD");
        SmartDashboard.setPersistent(prefix + "kI");
        SmartDashboard.setPersistent(prefix + "kV");
        SmartDashboard.setPersistent(prefix + "kS");
        SmartDashboard.setPersistent(prefix + "MaxOutput");
        SmartDashboard.setPersistent(prefix + "MinOutput");
        SmartDashboard.setPersistent(prefix + "RpmDelta");
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
        rpmDelta = SmartDashboard.getNumber(prefix + "RpmDelta", DeltaRPM);

        setConfig();

        double vel = SmartDashboard.getNumber(prefix + "Set RPM", DefaultVelocityRPM);
        if (vel != targetVelocity)
        {
            targetVelocity = vel;
            targetVelocityChanged = true;

            // if (velocityRPM == 0)
            //     stop();
            // else
            //     run();
        }

        //TODOTODO!!!
        // if (velocityRPM < 50)
        //     velocityRPM = 50;

        pidCtrl.pid(kP, kD, kI);
    }
}

/*
    //TODO Param?
    public void run(double target) {
        ++execCounter;
        int dtime = (int) (execCounter * 1000 * Constants.TimePeriod);

        double t = Timer.getFPGATimestamp();
        double dt = time > 0 ? t - time : 0;
        time = t;

//        downSetRPM = upSetRPM;
        double velUp = upEncoder.getVelocity();
        double velDown = downEncoder.getVelocity();

        double pos = upEncoder.getPosition();
        double pos1 = downEncoder.getPosition();
        //double pos2 = downEncoderR.getPosition();
        // double rpm = SmartDashboard.getNumber("RPM", 500);
        // if (targetRPM != rpm)
        //     targetRPM = rpm;
        // if (targetRPM < 50)
        //     targetRPM = 500;
//        System.out.printf("VEL, %.2f, -, %.2f%n", velUp, velDown);

        if (controlValueUp < controlOutputMin) {
            controlValueUp = controlOutputMin;
            controlValueDown = -controlOutputMin;
            System.out.printf("VEL, %.2f, -, %.2f%n", velUp, velDown);

            upShooterMotor.set(controlValueUp);
            downShooterMotor.set(controlValueDown);
            downShooterMotorR.set(-controlValueDown);
            System.out.printf("Applied Output: %.3f, %.3f, %.4f %n", controlValueUp, controlValueDown, upShooterMotor.getAppliedOutput());
            return;
        }

        double difUp = kV * upSetRPM + upPidCtrl.calculateDif(velUp, upSetRPM, dt);
//        System.out.printf("calculateDif: %.3f, %.3f, %.2f, -, %.2f%n", velDown, downSetRPM, velUp, velDown);
        double difDown = kV * (-downSetRPM) + downPidCtrl.calculateDif(-velDown, -downSetRPM, dt);

        atSpeed = Math.abs(difUp / upSetRPM - 1) < posDelta && Math.abs(difDown / downSetRPM - 1) < posDelta;

        controlValueUp += difUp / upSetRPM;
        controlValueDown += difDown / downSetRPM;

        double ctrlvalUp = controlValueUp;
        double ctrlvalDown = controlValueDown;
    
        controlValueUp = PIDCtrl.limitSignedRange(controlValueUp, controlOutputMin, controlOutputMax);
        controlValueDown = PIDCtrl.limitSignedRange(controlValueDown, -controlOutputMax, -controlOutputMin);

//        upShooterMotor.setVoltage(ctrlSpeed);
        upShooterMotor.set(controlValueUp);
//        downShooterMotor.set(-controlValueUp);
        downShooterMotor.set(controlValueDown);
        downShooterMotorR.set(-controlValueDown);

        double cur1 = upShooterMotor.getOutputCurrent();
        double cur2 = downShooterMotor.getOutputCurrent();
        double cur3 = downShooterMotorR.getOutputCurrent();

        SmartDashboard.putNumber("Shooter RPM Up", velUp);
        SmartDashboard.putNumber("Shooter RPM Down", velDown);
        SmartDashboard.putNumber("Thrust", controlValueUp);

        SmartDashboard.putNumber("CurrentUp", cur1);
        SmartDashboard.putNumber("CurrentDown", cur2);

//        System.out.printf("CUR, %.3f, %.3f, %.3f%n", cur1, cur2, cur3);

        System.out.printf("SHT, %d, %.2f, -, %.2f, %.3f, -, %.4f, %.4f, %.4f, %.4f%n", dtime, 1000 * dt, pos, velUp, difUp, ctrlvalUp, controlValueUp, upShooterMotor.getAppliedOutput());
        System.out.printf("SHT, %d, %.2f, -, %.2f, %.3f, -, %.4f, %.4f, %.4f, %.4f, %.4f%n", dtime, 1000 * dt, pos, velDown, difDown, ctrlvalDown, controlValueDown, downShooterMotor.getAppliedOutput(), downShooterMotorR.getAppliedOutput());
    }

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
        double coeff = SmartDashboard.getNumber("Coeff", 1);
        //ctrlSpeed *= coeff;
//        System.out.println("Speed: " + speed);
        double velUp = upEncoder.getVelocity();
        double velDown = downEncoder.getVelocity();

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
        SmartDashboard.putNumber("Shhot RPM Up", vel);
        SmartDashboard.putNumber("Shhot RPM Up", vel);

        System.out.println("Applied Output: " + upShooterMotor.getAppliedOutput());            
    }
*/