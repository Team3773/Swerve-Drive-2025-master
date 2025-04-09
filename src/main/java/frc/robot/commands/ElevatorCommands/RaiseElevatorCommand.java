// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RaiseElevatorCommand extends InstantCommand {
  Double requestedHeight;

  ElevatorSubsystem elevatorSubsystem;
  public RaiseElevatorCommand(ElevatorSubsystem elevatorSubsystem, Double requestedHeight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
    this.requestedHeight = requestedHeight;
    addRequirements(elevatorSubsystem);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    this.elevatorSubsystem.goToPosition(this.requestedHeight);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.elevatorSubsystem.goToPosition(this.requestedHeight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.elevatorSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.elevatorSubsystem.isAtHeight(requestedHeight);
  }
}
