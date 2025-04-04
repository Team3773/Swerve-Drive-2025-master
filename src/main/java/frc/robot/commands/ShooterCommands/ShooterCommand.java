// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */
    ShooterSubsystem shooterSubsystem;
    BooleanSupplier bumperSupplier, triggerSupplier, speedIncrease;
    
  public ShooterCommand(ShooterSubsystem shooterSubsystem, BooleanSupplier bumper, BooleanSupplier trigger, BooleanSupplier speedIncrease) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    this.bumperSupplier = bumper;
    this.triggerSupplier = trigger;
    this.speedIncrease = speedIncrease;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

// Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.checkBeamAndControlMotor(this.bumperSupplier.getAsBoolean(), this.triggerSupplier.getAsBoolean(), this.speedIncrease.getAsBoolean());
  }

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