package frc.robot;

import com.ctre.phoenix6.configs.Pigeon2Configuration;

public final class RobotConfig {
    public final String systemCANBus;
    // Name of CAN bus that the devices are located on;
    public final String driveCANBus;
    public final int pigeonId;
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    public final Pigeon2Configuration pigeonConfigs;
    public final SwerveModuleConfig frontLeft;
    public final SwerveModuleConfig frontRight;
    public final SwerveModuleConfig backLeft;
    public final SwerveModuleConfig backRight;

    public static final double HalfSwerveOffset = 11.0;
    public static final double FuelHalfSwerveOffset = 11.125;
    public static final boolean InvertLeftSide = false;
    public static final boolean InvertRightSide = true;

    public static final RobotConfig FuelRobot = new RobotConfig(
        "Drivetrain", "Mech", 20,
        new SwerveModuleConfig(2, 1, 21, 0.105224609375, FuelHalfSwerveOffset, FuelHalfSwerveOffset, InvertLeftSide, false, false),
        new SwerveModuleConfig(4, 3, 22, -0.12060546875, FuelHalfSwerveOffset, -FuelHalfSwerveOffset, InvertRightSide, false, false),
        new SwerveModuleConfig(6, 5, 23, -0.466796875, -FuelHalfSwerveOffset, FuelHalfSwerveOffset, InvertLeftSide, false, false),
        new SwerveModuleConfig(8, 7, 24, -0.037109375, -FuelHalfSwerveOffset, -FuelHalfSwerveOffset, InvertRightSide, false, false));

    public static final RobotConfig CoralRobot = new RobotConfig(
        "CANdace", "rio", 24,
        new SwerveModuleConfig(7, 8, 23, -0.11669921875, 11.5, 11.5, InvertLeftSide, false, false),
        new SwerveModuleConfig(1, 2, 20, -0.291015625, 11.5, -11.5, InvertRightSide, false, false),
        new SwerveModuleConfig(5, 6, 22, 0.048828125, -11.5, 11.5, InvertLeftSide, false, false),
        new SwerveModuleConfig(3, 4, 21, -0.371826171875, -11.5, -11.5, InvertRightSide, false, false));

    public static final RobotConfig AlgaeRobot = new RobotConfig(
        "Drivetrain", "rio", 20,
        new SwerveModuleConfig(1, 2, 24, -0.03173828125, 11.5, 11.5, InvertLeftSide, false, false),
        new SwerveModuleConfig(19, 18, 23, -0.455322265625, 11.5, -11.5, InvertRightSide, false, false),
        new SwerveModuleConfig(4, 3, 25, -0.12744140625, -11.5, 11.5, InvertLeftSide, false, false),
        new SwerveModuleConfig(16, 17, 26, 0.115478515625, -11.5, -11.5, InvertRightSide, false, false));

    public RobotConfig(String driveCANBus, String systemCANBus, int pigeonId,
        SwerveModuleConfig fl, SwerveModuleConfig fr, SwerveModuleConfig bl, SwerveModuleConfig br) {
        this.driveCANBus = driveCANBus;
        this.systemCANBus = systemCANBus;
        this.pigeonId = pigeonId;
        pigeonConfigs = null;
        frontLeft = fl;
        frontRight = fr;
        backLeft = bl;
        backRight = br;
    }
}
