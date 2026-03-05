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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants;
import frc.robot.PIDCtrl;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX motor;
    private final TalonFX follower;

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
    /* Start at velocity 0, use slot 1 */
    private final VelocityTorqueCurrentFOC velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(1);
    /* Keep a neutral out so we can disable the motor */
    private final NeutralOut brake = new NeutralOut();

    // private final RelativeEncoder upEncoder;
    // private final RelativeEncoder downEncoder;

    private final PIDCtrl upPidCtrl;
    private final PIDCtrl downPidCtrl;
    private double upSetRPM = ShooterConstants.Speed;
    private double downSetRPM = ShooterConstants.SpeedDown;

    private double timeDelta = Constants.TimePeriod;

    RelativeEncoder m_alternateEncoder;
    SparkClosedLoopController closedLoopCtrl;
    //private SparkPIDController m_pidController;
    private PIDController m_pidController;

    // PID coefficients
    public double kP = ShooterConstants.kP;
    public double kD = ShooterConstants.kD;
    public double kI = ShooterConstants.kI;
    public double kV = ShooterConstants.kV;
    public double controlOutputMax = ShooterConstants.ControlOutputMax;
    public double controlOutputMin = ShooterConstants.ControlOutputMin;
    public double posDelta = ShooterConstants.PositionDelta;
    public double rpmDelta = ShooterConstants.RPMDelta;

    // public double kIz = 0;
    // public double kFF= 0;
    // public double kMaxOutput= 1;
    // public double kMinOutput = -1;

    private int execCounter = 0;
    private double time = 0;
    private double controlValueUp = 0;
    private double controlValueDown = 0;
    private boolean atSpeed = false;

//    private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
    // private SparkMaxConfig motorConfig;
    // private SparkLimitSwitch forwardLimitSwitch;
    // private SparkLimitSwitch reverseLimitSwitch;
    

    /**
     * This subsytem that controls the roller.
     */
    public ShooterSubsystem(CANBus canBus) {
        motor = new TalonFX(Constants.ShooterConstants.RightMotorId, canBus);
        int followerId = 0;//Constants.ShooterConstants.RightMotorId;
        follower = followerId > 0 ? new TalonFX(followerId, canBus) : null;

        TalonFXConfiguration configs = new TalonFXConfiguration();

        /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
        configs.Slot0.kS = 0.01; // To account for friction, add 0.1 V of static feedforward
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
        // if (follower != null)
        //     follower.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Opposed));

        m_pidController = new PIDController(kP, kI, kD);
        //m_pidController.setIZone(kIz);
        m_pidController.setSetpoint(1);

    // set PID coefficients
    // m_pidController.setIZone(kIz);
    // m_pidController.setFF(kFF);
    // m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        upPidCtrl = new PIDCtrl(kP, kD, kI, timeDelta);
        downPidCtrl = new PIDCtrl(kP, kD, kI, timeDelta);
    }

    public void shoot(double distance)
    {
        // Set speed
        // Set hood angle
        // Wait
        // Run indexer
    }

    public void shoot()
    {
    }

    public void shootOnce()
    {
    }

    @Override
    public void periodic() {
    }

    public boolean atSetPoint() {
        return atSpeed;
    }

    public void init(double upRPM, double downRPM) {
        System.out.println("Shooter Initializing");
        getParams();
        upSetRPM = upRPM;
        downSetRPM = downRPM;
        execCounter = 0;
        
        time = 0;
        controlValueUp = 0;
        controlValueDown = 0;
        atSpeed = false;

        // upEncoder.setPosition(0);
        // downEncoder.setPosition(0);

        // upPidCtrl.reset();
        // downPidCtrl.reset();
        // m_pidController.reset();
        // m_pidController.setSetpoint(10);
    }

    public void setSpeed(double speed) {
        speed = -speed;
        double rps = speed  * Constants.ShooterConstants.MaxMotorRPS;

        motor.setControl(velocityVoltage.withVelocity(speed * Constants.ShooterConstants.MaxMotorRPS));
//        .withFeedForward(feedforward))
//        motor.setControl(velocityTorque.withVelocity(speed * Constants.MaxMotorRPS));
        double v = motor.getVelocity().getValue().magnitude();
        System.out.println("Speed: " + speed + " / " + rps + " / " + v);
    }

    public void stop() {
        motor.setControl(brake);
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

    public void runRaw(double target) {
        // double velUp = upEncoder.getVelocity();
        // double velDown = downEncoder.getVelocity();
        // upShooterMotor.set(target);
        // downShooterMotor.set(-target);
        // SmartDashboard.putNumber("Shooter RPM Up", velUp);
        // SmartDashboard.putNumber("Shooter RPM Down", velDown);
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
*/

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
    public void putParams() {
        SmartDashboard.putNumber("Set RPM Up", upSetRPM);
//        SmartDashboard.putNumber("Set RPM Down", downSetRPM);

        SmartDashboard.putNumber("kP_Sh", kP);
        SmartDashboard.putNumber("kD_Sh", kD);
        SmartDashboard.putNumber("kI_Sh", kI);
        SmartDashboard.putNumber("kV_Sh", kV);
        SmartDashboard.putNumber("MaxOutput_Sh", controlOutputMax);
        SmartDashboard.putNumber("MinOutput_Sh", controlOutputMin);
        SmartDashboard.putNumber("PosDelta_Sh", posDelta);
        SmartDashboard.putNumber("RpmDelta_Sh", rpmDelta);
    }

    public void getParams() {
        upSetRPM = SmartDashboard.getNumber("Set RPM Up", ShooterConstants.Speed);
//        downSetRPM = SmartDashboard.getNumber("Set RPM Down", ShooterConstants.SpeedDown);

        kP = SmartDashboard.getNumber("kP_Sh", ShooterConstants.kP);
        kD = SmartDashboard.getNumber("kD_Sh", ShooterConstants.kD);
        kI = SmartDashboard.getNumber("kI_Sh", ShooterConstants.kI);
        kV = SmartDashboard.getNumber("kV_Sh", ShooterConstants.kV);
        controlOutputMax = SmartDashboard.getNumber("MaxOutput_Sh", ShooterConstants.ControlOutputMax);
        controlOutputMin = SmartDashboard.getNumber("MinOutput_Sh", ShooterConstants.ControlOutputMin);
        posDelta = SmartDashboard.getNumber("PosDelta_Sh", ShooterConstants.PositionDelta);
        rpmDelta = SmartDashboard.getNumber("RpmDelta_Sh", ShooterConstants.RPMDelta);

        //TODOTODO!!!
        if (upSetRPM < 50)
            upSetRPM = 50;
        if (downSetRPM > -50)
            downSetRPM = -50;

        upPidCtrl.pid(kP, kD, kI);
        downPidCtrl.pid(kP, kD, kI);
    }
}