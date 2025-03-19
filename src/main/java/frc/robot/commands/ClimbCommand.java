// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command {
  BooleanSupplier button1, button2, button3, ratchetButton, ratchetRelease;
  ClimbSubsystem climbSubsystem;
  double startPosition = 0.0;
  double lowerPosition = -0.35;
  double climbPosition = 0.25;

  public ClimbCommand(BooleanSupplier button1, BooleanSupplier button2, BooleanSupplier button3,
      BooleanSupplier ratchetButton, BooleanSupplier ratchetRelease, ClimbSubsystem climbSubsystem) {
    this.button1 = button1; //driver back
    this.button2 = button2; //driver start
    this.button3 = button3; //driver dpadUp
    this.ratchetButton = ratchetButton; //driver dpadDown
    this.ratchetRelease = ratchetRelease; //driver Y
    this.climbSubsystem = climbSubsystem;
    addRequirements(climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    this.climbSubsystem.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (button1.getAsBoolean()) {
      // Go to Climb Pull position
      this.climbSubsystem.goToPosition(lowerPosition);
    } else if (button2.getAsBoolean()) {
      // Go to Climb Hook Position
      this.climbSubsystem.goToPosition(climbPosition);
    } else if (button3.getAsBoolean()) {
      //Go to Zero Position
      this.climbSubsystem.goToPosition(startPosition);
    }

     if (ratchetButton.getAsBoolean()) {
      this.climbSubsystem.runRatchet();
    } else if (ratchetRelease.getAsBoolean()) {
      this.climbSubsystem.releaseRatchet();
    } else {
      this.climbSubsystem.stopRatchet();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}