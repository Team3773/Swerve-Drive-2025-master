package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {

        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        // public static final double kDrivingMotorReduction = 1 / 12.8;
        // public static final double kTurningMotorGearRatio = 1 / 18.0;

        // Gear Ratios
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth
        // will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;
        // 45 teeth on the wheel's bevel gear, 16 teeth on the first-stage spur gear, 19
        // teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 19.0) / (kDrivingMotorPinionTeeth * 15);
        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        //
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = 5676 / 60;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 40; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps

    }

    public static final class DriveConstants {
        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(28);
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(32);

        public static final double kDriveRadius = Math.hypot(kTrackWidth / 2, kWheelBase / 2);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
        // public static final SwerveDriveKinematics kDriveKinematics = new
        // SwerveDriveKinematics(
        // new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        // new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        // new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        // new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 4;
        public static final int kFrontRightDriveMotorPort = 8;
        public static final int kBackRightDriveMotorPort = 11;
        public static final int kBackLeftDriveMotorPort = 3;

        public static final int kFrontLeftTurningMotorPort = 5;
        public static final int kFrontRightTurningMotorPort = 9;
        public static final int kBackRightTurningMotorPort = 10;
        public static final int kBackLeftTurningMotorPort = 2;

        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // public static final boolean kFrontLeftTurningEncoderReversed = false;
        // public static final boolean kBackLeftTurningEncoderReversed = false;
        // public static final boolean kFrontRightTurningEncoderReversed = false;
        // public static final boolean kBackRightTurningEncoderReversed = false;

        // public static final boolean kFrontLeftDriveEncoderReversed = false;
        // public static final boolean kBackLeftDriveEncoderReversed = false;
        // public static final boolean kFrontRightDriveEncoderReversed = false;
        // public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftTurningAbsoluteEncoderPort = 6;
        public static final int kFrontRightTurningAbsoluteEncoderPort = 7;
        public static final int kBackRightTurningAbsoluteEncoderPort = 12;
        public static final int kBackLeftTurningAbsoluteEncoderPort = 1;

        // public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        // public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        // public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        // public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 1.03;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 2.46;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -0.83;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -1.96;

        public static final double kMaxSpeedMetersPerSecond = 6.36;
        public static final double kMaxAngularSpeed = 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kMaxAngularSpeed / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kMaxAngularSpeed / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static final double kDirectionSlewRate = 4; // radians per second was 1.2 4
        public static final double kMagnitudeSlewRate = 3; // percent per second (1 = 100%) 2.5
        public static final double kRotationalSlewRate = 5; // percent per second (1 = 100%) 10

        public static final HolonomicPathFollowerConfig kPathFollowerConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig,
                                                                                                               // this
                                                                                                               // should
                                                                                                               // likely
                                                                                                               // live
                                                                                                               // in
                                                                                                               // your
                                                                                                               // Constants
                                                                                                               // class
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                kMaxSpeedMetersPerSecond, // Max module speed, in m/s
                kDriveRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig()); // Default path replanning config. See the API for the options here) TODO:
                                         // enable replanning
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kMaxAngularSpeed / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);

        // constant for aligning to an angle while driving
        public static final double kDriveWhileAligningP = .01;

    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;
        public static final boolean kUseQuadraticInput = true;
    }

    public final class GyroConstants {
        public static final boolean kGyroReversed = true;
        public static final double kGyroAngularOffset = Math.PI / 2;
        public static final double kGyroAngularOffsetDeg = Math.toDegrees(kGyroAngularOffset);
    }
}
