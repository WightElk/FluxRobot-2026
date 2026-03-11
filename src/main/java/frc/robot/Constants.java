// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int Forward = 0;
  public static final int Backward = 0;

  public static final String fieldLayoutFile = "2026-rebuilt-welded.json";
  //"2025-reefscape-andymark.json";

  public static final double robotLength = 34;

  public static int TimePeriodMsec = 20;  // in seconds
  public static double TimePeriod = 0.001 * TimePeriodMsec;  // in milliseconds

  public static final double MaxMotorRPM = 6000.0;  //6380
  public static final double MaxMotorRPS = 0.5 * MaxMotorRPM / 60.0;

  public static final class DriveConstants {
    public static final double AutoModeSpeed = 0.8;  // Move backward to driver station
    public static final double AutoModeDriveTime = 2.0;  // In seconds
    public static final double AutoModeDriveTime_Max = 10.0;  // In seconds

    public static final double Holo_X_kP = 0.5;
    public static final double Holo_X_kI = 0.0;
    public static final double Holo_X_kD = 0.05;
    public static final double Holo_Y_kP = 0.5;
    public static final double Holo_Y_kI = 0.0;
    public static final double Holo_Y_kD = 0.05;
    public static final double Holo_Rot_kP = 1.0;
    public static final double Holo_Rot_kI = 0.1;
    public static final double Holo_Rot_kD = 0.4;
    public static final Pose2d Path_Tolerance = new Pose2d(0.05, 0.05, new Rotation2d(Math.toRadians(5)));
    public static final Pose2d Target_Tolerance = new Pose2d(0.04, 0.04, new Rotation2d(Math.toRadians(3)));

    /** Meters per Second */
    public static final double maxSpeed = 2;//5.5;
  }

  public static final class IntakeConstants {
    public static final int MotorId = 1;
    public static final int FollowerId = 2;//Absent
    public static final int TiltMotorId = 20;
    public static final double InSpeed = -40;

    public static final double TiltStep = 2;

    public static final double OutSpeed = -0.2;

    public static final double MaxMotorRPS = 0.5 * MaxMotorRPM / 60.0;

    public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
    public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
  }

  public static final class IndexerConstants {
    public static final int IndexerId = 4;
    public static final int FeederId = 3;
    public static final int LeftFeederId = 5;
    public static final int RightFeederId = 6;

    public static final double MaxMotorRPS = 0.5 * MaxMotorRPM / 60.0;

    public static final double InSpeed = -50;//1.5;//ROLLER_ALGAE_IN
    public static final double FeederSpeed = 60;
    public static final double BackwardSpeed = -3000;//-1.5;//ROLLER_ALGAE_OUT
    public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
    public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
  }

  public static final class ShooterConstants {
    public static final int LeftMotorId = 10;
    public static final int RightMotorId = 11;
    public static final int HoodMotorId = 12;

    public static final int SHOOT_MOTOR_CURRENT_LIMIT = 40;
    public static final double SHOOT_MOTOR_VOLTAGE_COMP = 10;

    public static final double MaxMotorRPS = 1.0 * MaxMotorRPM / 60.0;

    public static final double ALGAE_IN = -0.4;
    public static final double ALGAE_OUT = 0.2;

    public static final double SHOOT_ALGAE_IN = 0.4;
    public static final double SHOOT_ALGAE_OUT = -0.8;

    public static final int ShootWaitDelay = 200;
    public static final int ShootStartDelay = 1500;
    public static final int ShootFinishDelay = 1000;
  
    public static final int IntakeBackTime = 300;
    public static final int ShootWaitTime = IntakeBackTime + ShootWaitDelay;
    public static final int ShootStartTime = ShootWaitTime + ShootStartDelay;
    public static final int ShootFinishTime = ShootStartTime + ShootFinishDelay;
  
    public static final double Speed = 90;//100;
    public static final double SpeedDown = -2500;

    public static final double HoodStep = 1;

    public static final double SpeedUp1 = 2110;
    public static final double SpeedUp2 = 2200;
    public static final double SpeedUp3 = 2800;
    public static final double SpeedUp4 = 3000;

    public static final double SpeedDown1 = -1950;
    public static final double SpeedDown2 = -2200;
    public static final double SpeedDown3 = -2800;
    public static final double SpeedDown4 = -3000;

    //PID2: 0.02/0.008/0.0008

    //2500/0.4/0.00008/0.001500
    //3500/2500/0.7/0.00008/0.100 / 0.002
    //1500/0.3/0.00003/0.100 / 0.001

    //PID1: 0.4/0.00008/0.001
    public static final double kP = 0.4;//0.2
    public static final double kD = 0.00008;//0.001
    public static final double kI = 0.001;
    public static final double kV = 0.0;

    public static final double ControlOutputMax = 1.0;
    public static final double ControlOutputMin = 0.1;

    public static final double PositionDelta = 100;  // Relative
    public static final double RPMDelta = 100;  // Absolute
  }

    public static final class ElevatorConstants {
    public static final int LeaderId = 1;
    public static final int FollowerId = 2;

    public static final double BottomPos = 0.05;
    public static final double Level1pos = -7;
    public static final double Level2pos = -19.6;

    public static final double JogStep = 0.2;

    public static final double kP = 0.05;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double OutputLimit = 0.3;
  }

  public static final class TrayConstants {
    public static final int MotorId = 3;
    public static final int FeedMotorId = 4;
    public static final int CanDiId = 7;
  
    public static final double InSpeed = 0.5;
    public static final double OutSpeed = 1.0;
    public static final double Speed = 0.8;
    public static final double BackwardSpeed = 0.5;
  }

  public static final class PneumoConstants {
    public static final int HubId = 3;
    public static final int ForwardLeftSolenoidId = 6;
    public static final int ReverseLeftSolenoidId = 7;
    public static final int ForwardRightSolenoidId = 0;
    public static final int ReverseRightSolenoidId = 3;

    public static final double MinPressure = 20;
    public static final double MaxPressure = 60;
  }

  public static final class RollerConstants {
    public static final int MotorId = 14;
    public static final int FollowerId = 13;
    public static final double InSpeed = 1.0;
    public static final double OutSpeed = -0.5;

    public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
    public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
  }

  public static class OperatorConstants {
    public static final int DriverControllerPort = 0;
    public static final int OperatorControllerPort = 1;

    public static final boolean UseTwoControllers = true;

    public static final double TriggerThreshold = 0.05;

    public static final double xStartPos = 0.09;
    public static final double xMiddlePos = 0.6;
    public static final double yStartPos = 0.1;
    public static final double yMiddlePos = 0.3;
    public static final double yMaxPos = 0.8; 

    public static final double xStartRot = 0.09;
    public static final double xMiddleRot = 0.6;
    public static final double yStartRot = 0.1;
    public static final double yMiddleRot = 0.5;
    public static final double yMaxRot = 1.0;

    public static final double LinCoef = 0.15;
    public static final double Threshold = 0.0;
    public static final double CuspX = 0.9;
    //TODO: Tune before competition!
    public static final double SpeedLimitX = 0.5;
    public static final double MinLimit = 0.4;

    public static final double RotLinCoef = 0.2;
    public static final double RotThreshold = 0.0;
    public static final double RotCuspX = 0.5;
    //TODO: Tune before competition!
    public static final double SpeedLimitRot = 0.8;
  }

  /**
   * Vision system constants (Coral robot only - single camera).
   */
  public static final class VisionConstants {
    /** PhotonVision camera name (must match name in PhotonVision UI) */
    public static final String CAMERA_NAME = "Logitech_Webcam_C930e";
    public static final String CameraBackName = "ThriftyCam";

    public static final Transform3d robotToCam1 = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
    public static final Transform3d robotToCam2 = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

    public static double ReefPolesGap = Units.feetToMeters(13.0);

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    // Distance to AprilTag in inches
    public static final double TargetDistance = 27;

    /** P-controller gain for rotation alignment */
    public static final double ROTATION_P = 0.07;

    /** P-controller gain for forward drive control */
    public static final double DRIVE_P = 0.15;//0.1

    /** Angle tolerance for alignment completion (degrees) */
    public static final double ANGLE_TOLERANCE = 2.0;

    /** Target area percentage for desired distance (~1.5-2m away) */
    public static final double AREA_TARGET = 8.0;

    /** Area tolerance for distance completion */
    public static final double AREA_TOLERANCE = 1.0;

    /** Minimum rotation speed (rad/s) */
    public static final double MIN_ROTATION_SPEED = 0.1;

    /** Maximum rotation speed (rad/s) */
    public static final double MAX_ROTATION_SPEED = 2.0;

    /** Minimum drive speed (m/s) */
    public static final double MIN_DRIVE_SPEED = 0.2;

    /** Maximum drive speed (m/s) */
    public static final double MAX_DRIVE_SPEED = 1.5;

    /** Maximum yaw error before stopping forward drive (degrees) */
    public static final double MAX_YAW_ERROR_FOR_DRIVE = 15.0;

    public static final Pose3d cameraPoseFront = new Pose3d (
      new Translation3d(0.0, 0.0, 0.0), 
      new Rotation3d(0.0, 0.0, 0.0)
    );

    public static final Pose3d cameraPoseRear = new Pose3d (
      new Translation3d(0.0, 0.0, 0.0), 
      new Rotation3d(0.0, 0.0, Math.PI)
    );

    public static final double sizeX = 23.0;
    public static final double sizeY = 23.0;

    public static final double distanceToTag = 1.0;

    public static enum Direction {
      Center,
      Left,
      Right
    };
  }

  public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 2.0;
    public static final double kPYController = 2.0;
    public static final double kPThetaController = 3.3;

    public static HashMap<String, Command> eventMap = new HashMap<>();

    public static HashMap<List<PathPlannerTrajectory>, Command> gyroResets = new HashMap<>();

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final int CommandCount = 2;
    // Arrays of pairs: { Command name, Path name }
    public static final String[][] commands = {
//      {"Left", "LeftPath"},
//      {"Center", "LeftPath"},
//      {"Right", "LeftPath"}
    };
  }

  public static class LightConstants {
    public static final int CanId = 6;
  }
}
