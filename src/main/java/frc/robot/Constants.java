package frc.robot;

import java.util.spi.CurrencyNameProvider;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants {
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final double LENGTH = 20;
    public static final double WIDTH = 15;
    public static final double MOI = (1/12) * ROBOT_MASS * ((LENGTH * LENGTH) + (WIDTH* WIDTH));
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    // public static ModuleConfig config = new ModuleConfig(null, null, 0, null, null, 0);
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
        public static final double kDriveGearRatio = 6.75;
        public static final double kTurningMotorGearRatio = 150 /7;
        public static final int kDrivePPR = 42;
        public static final int kTurningEncoderCPR = 4096;
        public static final int kTurningEncoderPPR = kTurningEncoderCPR /4;
        public static final double kmaxSpeed = 1.5;
        public static final double kWheelCOF = .5; //This needs to be updated with an accurate coefficient of friction for the wheels
        public static final double currentLimit = 0.1; //Update this with an accurate current limit.
        //How many motors and which type of motor are being used?
        // Replace DCMotor.getNEO(1) with something accurate
        // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/system/plant/DCMotor.html
        public static final ModuleConfig moduleConfig = new ModuleConfig(kWheelDiameterMeters, kmaxSpeed,kWheelCOF,DCMotor.getNEO(1),kDriveGearRatio,currentLimit, 1);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final class ElevatorConstants {
        public static final int CAN_ID = 13;
        public static final int stepValue = 5;
    }

    public static final class ArmConstants {
        public static final int CAN_ID = 14;
    }

    public static final class ShooterConstants {
        public static final int LEFT_CAN_ID = 15;
        public static final int RIGHT_CAN_ID = 16;
    }

    public static final class ClimbConstants {
        public static final int CAN_ID = 17;
    }

}
