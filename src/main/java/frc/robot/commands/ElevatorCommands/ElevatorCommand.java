// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommand extends Command {
  /** Creates a new ElevatorCommand. */
  Double loadingHeight = 21.0;
  Double unStuckHeight = 26.0;
  Double level2Height = 48.5;
  Double level3Height = 84.5;

  BooleanSupplier loading, unStuck, level2, level3, goToBottom;



  ElevatorSubsystem elevatorSubsystem;
  public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, BooleanSupplier loading, BooleanSupplier unStuck, BooleanSupplier level2, BooleanSupplier level3, BooleanSupplier goToBottom) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
    this.loading = loading;
    this.unStuck = unStuck;
    this.level2 = level2;
    this.level3 = level3;
    this.goToBottom = goToBottom;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // this.elevatorSubsystem.goToPosition(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(goToBottom.getAsBoolean()){
      this.elevatorSubsystem.goToPosition(-150);
    }
  

    if(loading.getAsBoolean()){
      this.elevatorSubsystem.goToPosition(loadingHeight);
    }else if(unStuck.getAsBoolean()){
      this.elevatorSubsystem.goToPosition(unStuckHeight);
    }else if(level2.getAsBoolean()){
      this.elevatorSubsystem.goToPosition(level2Height);
    }else if(level3.getAsBoolean()){
      this.elevatorSubsystem.goToPosition(level3Height);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.elevatorSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}