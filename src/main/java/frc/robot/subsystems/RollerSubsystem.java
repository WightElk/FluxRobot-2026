package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkRelativeEncoder;
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
import frc.robot.Constants.RollerConstants;
import frc.robot.PIDCtrl;

//import frc.robot.PIDCtrl;

public class RollerSubsystem extends SubsystemBase {

    private SparkMax rollerMotor;
    private SparkMax rollerMotorR;
//    private final SparkMax upShooterMotor;
    // private SparkMax upShooterMotorR;
    // private SparkMax downShooterMotor;
    // private SparkMax downShooterMotorR;

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
    private SparkMaxConfig motorConfig;
    private SparkLimitSwitch forwardLimitSwitch;
    private SparkLimitSwitch reverseLimitSwitch;
    private RelativeEncoder encoder;

    private final PIDCtrl pidCtrl;
    private double timeDelta;
    private double controlValue;
    /**
     * This subsytem that controls the roller.
     */
    public RollerSubsystem () {
        timeDelta = 0.02;
        controlValue = 0;

    // Set up the roller motor as a brushed motor
    rollerMotor = new SparkMax(RollerConstants.MotorId, MotorType.kBrushless);
    rollerMotorR = new SparkMax(RollerConstants.FollowerId, MotorType.kBrushless);
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
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
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
    }

    public void init() {
        m_encoder.setPosition(0);
        // m_pidController.reset();
        // m_pidController.setSetpoint(10);
        pidCtrl.reset();
        controlValue = 0;
    }

    public void stop() {
        rollerMotor.set(0);
        rollerMotorR.set(0);
//        upShooterMotor.set(0);
        //upShooterMotor.stopMotor();
    }

    /**
     *  This is a method that makes the roller spin to your desired speed.
     *  Positive values make it spin forward and negative values spin it in reverse.
     * 
     * @param speedmotor speed from -1.0 to 1, with 0 stopping it
     */
    public void runRoller(double speed){
        rollerMotor.set(speed);
        rollerMotorR.set(-speed);
//        System.out.println("Roller: " + rollerMotor.getAppliedOutput() + " - " + rollerMotorR.getAppliedOutput());
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