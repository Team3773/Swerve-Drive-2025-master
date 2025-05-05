package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;

    public LimelightSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
    }

    public int getAprilTagID() {
        return (int) limelightTable.getEntry("tid").getDouble(-1);
    }

    public double[] getBotPose() {
        return limelightTable.getEntry("botpose").getDoubleArray(new double[6]);
    }

    // Get the horizontal offset (tx) from the Limelight camera
    public double getTX() {
        return limelightTable.getEntry("tx").getDouble(0.0);  // Default to 0.0 if no value
    }
    
    @Override
    public void periodic() {
       SmartDashboard.putBoolean("Has Target", hasTarget());
       SmartDashboard.putNumber("AprilTag ID", getAprilTagID());
}
}
   
   
    

