package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;

    public VisionSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public Pose2d getLimelightPose2d() {
        double[] botpose = limelightTable.getEntry("botpose").getDoubleArray(new double[6]);
        if (botpose.length < 6) return new Pose2d(); // fallback if no data
        return new Pose2d(
            new Translation2d(botpose[0], botpose[1]),
            Rotation2d.fromDegrees(botpose[5])
        );
    }

    public boolean hasValidTarget() {
        return limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
    }

    public double getTX() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    public double getTY() {
        return limelightTable.getEntry("ty").getDouble(0.0);
    }

    public double getTA() {
        return limelightTable.getEntry("ta").getDouble(0.0);
    }

    public double[] getBotPose() {
        return limelightTable.getEntry("botpose").getDoubleArray(new double[6]);
    }
}

