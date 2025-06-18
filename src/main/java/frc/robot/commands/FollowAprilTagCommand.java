// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class FollowAprilTagCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final LimelightSubsystem limelightSubsystem;

    private final double kPForward = 0.5;
    private final double kPRotate = 0.03;
    private final double targetDistance = 5; // meters

    public FollowAprilTagCommand(SwerveSubsystem swerveSubsystem, LimelightSubsystem limelightSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        addRequirements(swerveSubsystem, limelightSubsystem);
    }

    @Override
    public void execute() {
        if (!limelightSubsystem.hasTarget()) {
            // Stop the robot if no tag is detected
            swerveSubsystem.drive(new Translation2d(0.0, 0.0), 0.0, false);
            return;
        }

        double[] botPose = limelightSubsystem.getBotPose();
        double zDistance = botPose[2]; // Distance to the tag
        double xOffset = limelightSubsystem.getTX(); // Horizontal offset from the tag

        // Calculate forward and rotational speeds
        double forwardSpeed = (targetDistance - zDistance) * kPForward; // Reverse the sign
        double rotationSpeed = -xOffset * kPRotate;

        // Clamp values to prevent excessive speeds
        forwardSpeed = Math.max(Math.min(forwardSpeed, 0.5), -0.5);
        rotationSpeed = Math.max(Math.min(rotationSpeed, 0.3), -0.3);

        // Create a translation vector for forward movement
        Translation2d translation = new Translation2d(forwardSpeed, 0.0);

        // Drive the robot with the calculated translation and rotation
        swerveSubsystem.drive(translation, rotationSpeed, false);
    }

    @Override
    public void end(boolean interrupted) {
      swerveSubsystem.drive(new Translation2d(0.0, 0.0), 0.0, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
