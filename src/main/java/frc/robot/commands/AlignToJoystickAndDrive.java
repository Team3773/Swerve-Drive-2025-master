
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotShared;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.GyroConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// This aligns the robot to the joysticks absolute position rather than free rotation
public class AlignToJoystickAndDrive extends PIDCommand {
  static RobotShared m_robotShared = RobotShared.getInstance();
  private static SwerveSubsystem m_drive = m_robotShared.getDriveSubsystem();
  private static CommandXboxController m_driverController = m_robotShared.getDriverController(); 
  // isThereInput == 1 if true
  public AlignToJoystickAndDrive(double newXInput, double newYInput, boolean fieldRelative, boolean rateLimit, int isThereInput) {
    super(
      // The controller that the command will use
      new PIDController(AutoConstants.kDriveWhileAligningP * isThereInput, 0, 0),
      // This should return the measurement
      () -> m_drive.getHeading(), // replacing this with getRotation2d could mean I don't have to reverse the for loop
      // This should return the setpoint (can also be a constant)
      () ->
      (GyroConstants.kGyroAngularOffsetDeg + -Math.toDegrees(Math.atan2(-MathUtil.applyDeadband(newYInput, OIConstants.kDeadband),
             -MathUtil.applyDeadband(newXInput, OIConstants.kDeadband)))),
      // This uses the output
      output -> {
        m_drive.drive(
          -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDeadband), 
          -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDeadband), 
          output, fieldRelative, rateLimit, OIConstants.kUseQuadraticInput);
      }, m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(m_drive);
    getController().enableContinuousInput(-180, 180);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
