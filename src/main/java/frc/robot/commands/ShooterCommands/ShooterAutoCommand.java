// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterAutoCommand extends InstantCommand {
  Double leftmotorSpeed = 0.5;
  Double rightmotorSpeed = 1.5;

  ShooterSubsystem shooterSubsystem;


  public ShooterAutoCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (shooterSubsystem.beamBroken()) {
      shooterSubsystem.setLeftMotorSpeed(-leftmotorSpeed);
      shooterSubsystem.setRightMotorSpeed(rightmotorSpeed);
      System.out.println("Beam broken! Motor running.");
  } else {
      // Stop motors if the beam is broken
      shooterSubsystem.setLeftMotorSpeed(0.0);
      shooterSubsystem.setRightMotorSpeed(0.0);
      System.out.println("Beam Intact! Motor stopped.");
  }
  }

  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.runMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
