package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants {
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
        // Maximum speed of the robot in meters per second, used to limit acceleration.

    public static final class AutonConstants
    {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
        public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
    }

    public static final class DrivebaseConstants
    {
        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds
    }

    public static class OperatorConstants
    {
        // Joystick Deadband
        public static final double LEFT_X_DEADBAND  = 0.1;
        public static final double LEFT_Y_DEADBAND  = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT    = 6;
    }

    public static final class ModuleConstants {

        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveGearRatio = 12.8;
        public static final double kTurningMotorGearRatio = 18.0;
        public static final int kDrivePPR = 42;
        public static final int kTurningEncoderCPR = 4096;
        public static final int kTurningEncoderPPR = kTurningEncoderCPR / 4;
        public static final double kmaxSpeed = 1.5;
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
