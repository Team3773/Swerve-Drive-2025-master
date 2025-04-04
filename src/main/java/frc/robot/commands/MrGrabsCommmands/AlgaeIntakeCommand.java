// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MrGrabsCommmands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeIntakeCommand extends Command {
  /** Creates a new AlgaeIntakeCommand. */
    private final AlgaeIntakeSubsystem algaeIntakeSubsystem;
    private final BooleanSupplier xSupplier;
    private final BooleanSupplier ySupplier;

  public AlgaeIntakeCommand(AlgaeIntakeSubsystem algaeIntakeSubsystem, BooleanSupplier xSupplier, BooleanSupplier ySupplier ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.algaeIntakeSubsystem = algaeIntakeSubsystem;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    addRequirements(algaeIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (xSupplier.getAsBoolean()) {
      algaeIntakeSubsystem.startIntake();
  } else if (ySupplier.getAsBoolean()) {
      algaeIntakeSubsystem.reverseIntake();
  } else {
      algaeIntakeSubsystem.stopIntake();
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeIntakeSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
