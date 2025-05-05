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

    private final double kPForward = 0.05;
    private final double kPRotate = 0.02;
    private final double targetDistance = 1.0; // meters

    public FollowAprilTagCommand(SwerveSubsystem swerveSubsystem, LimelightSubsystem limelightSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        addRequirements(swerveSubsystem, limelightSubsystem);
    }

    @Override
    public void execute() {
        if (!limelightSubsystem.hasTarget()) {
          swerveSubsystem.drive(new edu.wpi.first.math.geometry.Translation2d(0.0, 0.0), 0.0, false); // stop if no tag
            return;
        }

        double[] botPose = limelightSubsystem.getBotPose();
        double zDistance = botPose[2];
        double xOffset = limelightSubsystem.getTX();

        double forwardSpeed = (zDistance - targetDistance) * kPForward;
        double rotationSpeed = -xOffset * kPRotate;

        // Clamp values
        forwardSpeed = Math.max(Math.min(forwardSpeed, 0.5), -0.5);
        rotationSpeed = Math.max(Math.min(rotationSpeed, 0.3), -0.3);

        // Drive: fwd, strafe, rotate, fieldRelative (set to false or true based on your preference)
        swerveSubsystem.drive(new Translation2d(forwardSpeed, 0.0), rotationSpeed, false);

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

