package frc.robot;


import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.FollowAprilTagCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;



public class RobotContainer
{

  // private static final Command PathPlannerPath = null;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(OIConstants.kDriverControllerPort);
  final CommandXboxController coDriverXbox = new CommandXboxController(OIConstants.kCoDriverControllerPort);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));

  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final AlgaeIntakeSubsystem algaeIntake = new AlgaeIntakeSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final FollowAprilTagCommand followAprilTagCommand = new FollowAprilTagCommand(drivebase, limelightSubsystem);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  // AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
  //                                                                () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
  //                                                                                              OperatorConstants.LEFT_Y_DEADBAND),
  //                                                                () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
  //                                                                                              OperatorConstants.LEFT_X_DEADBAND),
  //                                                                () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
  //                                                                                              OperatorConstants.RIGHT_X_DEADBAND),
  //                                                                driverXbox.getHID()::getYButtonPressed,
  //                                                                driverXbox.getHID()::getAButtonPressed,
  //                                                                driverXbox.getHID()::getXButtonPressed,
  //                                                                driverXbox.getHID()::getBButtonPressed);
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY(),
                                                                () -> driverXbox.getLeftX())
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.RIGHT_X_DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

   /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
  driverXbox::getRightY)
.headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.LEFT_X_DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);
  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  // Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
  //     () -> MathUtil.applyDeadband(driverXbox.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
  //     () -> MathUtil.applyDeadband(driverXbox.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
  //     () -> MathUtil.applyDeadband(driverXbox.getRightX() * -1, OperatorConstants.RIGHT_X_DEADBAND),
  //     () -> MathUtil.applyDeadband(driverXbox.getRightY() * -1, OperatorConstants.RIGHT_X_DEADBAND));

  // // Applies deadbands and inverts controls because joysticks
  // // are back-right positive while robot
  // // controls are front-left positive
  // // left stick controls translation
  // // right stick controls the angular velocity of the robot
  // Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
  //     () -> MathUtil.applyDeadband(driverXbox.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
  //     () -> MathUtil.applyDeadband(driverXbox.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
  //     () -> driverXbox.getRightX() * -1);

  // Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
  //     () -> MathUtil.applyDeadband(driverXbox.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
  //     () -> MathUtil.applyDeadband(driverXbox.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
  //     () -> driverXbox.getRawAxis(2) * -1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    // Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    // Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    // Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
    //     driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    // Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    // Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
    //     driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }    
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      // driverXbox.b().whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
      coDriverXbox.povDown().onTrue(Commands.runOnce(elevator::decrementPosition, elevator));
      coDriverXbox.povUp().onTrue(Commands.runOnce(elevator::incrementPosition, elevator));
      coDriverXbox.povLeft().onTrue(Commands.runOnce(climbSubsystem::decrementPosition, climbSubsystem));
      coDriverXbox.povRight().onTrue(Commands.runOnce(climbSubsystem::incrementPosition, climbSubsystem));
      elevator.setDefaultCommand(new ElevatorCommand(elevator,() -> coDriverXbox.a().getAsBoolean(),() -> coDriverXbox.y().getAsBoolean(),() -> coDriverXbox.b().getAsBoolean(), () -> coDriverXbox.x().getAsBoolean(), () -> coDriverXbox.leftBumper().getAsBoolean()));
      climbSubsystem.setDefaultCommand(new ClimbCommand(() -> coDriverXbox.back().getAsBoolean(),() -> coDriverXbox.start().getAsBoolean(),() -> coDriverXbox.leftBumper().getAsBoolean(),() -> coDriverXbox.start().getAsBoolean(), () -> driverXbox.y().getAsBoolean(), climbSubsystem));
      // coDriverXbox.leftTrigger().whileTrue(Commands.runOnce(armSubsystem::incrementPosition, armSubsystem));
      // coDriverXbox.leftBumper().whileTrue(Commands.runOnce(armSubsystem::decrementPosition, armSubsystem));
      shooterSubsystem.setDefaultCommand(new ShooterCommand(shooterSubsystem, coDriverXbox.rightBumper()::getAsBoolean, coDriverXbox.rightTrigger()::getAsBoolean, coDriverXbox.leftTrigger()::getAsBoolean));
    
      // drivebase.setDefaultCommand(       
      //     !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      // driverXbox.y().whileTrue(drivebase.aimAtSpeaker(2));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
      coDriverXbox.povDown().onTrue(Commands.runOnce(elevator::decrementPosition, elevator));
      coDriverXbox.povUp().onTrue(Commands.runOnce(elevator::incrementPosition, elevator));
      coDriverXbox.povLeft().onTrue(Commands.runOnce(climbSubsystem::decrementPosition, climbSubsystem));
      coDriverXbox.povRight().onTrue(Commands.runOnce(climbSubsystem::incrementPosition, climbSubsystem));
      elevator.setDefaultCommand(new ElevatorCommand(elevator,() -> coDriverXbox.a().getAsBoolean(),() -> coDriverXbox.y().getAsBoolean(), () -> coDriverXbox.b().getAsBoolean(), () -> coDriverXbox.x().getAsBoolean(), () -> coDriverXbox.leftBumper().getAsBoolean()));
      climbSubsystem.setDefaultCommand(new ClimbCommand(() -> driverXbox.back().getAsBoolean(),() -> driverXbox.start().getAsBoolean(),() -> driverXbox.povUp().getAsBoolean(), () -> driverXbox.y().getAsBoolean(), () -> driverXbox.povDown().getAsBoolean(), climbSubsystem));
      // coDriverXbox.leftTrigger().whileTrue(Commands.runOnce(armSubsystem::incrementPosition, armSubsystem));
      // coDriverXbox.leftBumper().whileTrue(Commands.runOnce(armSubsystem::decrementPosition, armSubsystem));
      shooterSubsystem.setDefaultCommand(new ShooterCommand(shooterSubsystem, coDriverXbox.rightBumper()::getAsBoolean, coDriverXbox.rightTrigger()::getAsBoolean, coDriverXbox.leftTrigger()::getAsBoolean));
      // drivebase.setDefaultCommand(
      //     !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(){
    // An example command will be run in autonomous
  //  try{
     return followAprilTagCommand;
   //  return new PathPlannerAuto("TestAuto");
  //  } catch (Exception e) {
     //   DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      //  return Commands.none();
   // }
} 

  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
// public Command getAutonomousCommand() {
// // 1. Create trajectory settings
// TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
// AutoConstants.kMaxSpeedMetersPerSecond,
// AutoConstants.kMaxAccelerationMetersPerSecondSquared)
// .setKinematics(DriveConstants.kDriveKinematics);

// // 2. Generate trajectory
// Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
// new Pose2d(0, 0, new Rotation2d(0)),
// List.of(
// new Translation2d(1, 0),
// new Translation2d(1, -1)),
// new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
// trajectoryConfig);

// // 3. Define PID controllers for tracking trajectory
// PIDController xController = new PIDController(AutoConstants.kPXController, 0,
// 0);
// PIDController yController = new PIDController(AutoConstants.kPYController, 0,
// 0);
// ProfiledPIDController thetaController = new ProfiledPIDController(
// AutoConstants.kPThetaController, 0, 0,
// AutoConstants.kThetaControllerConstraints);
// thetaController.enableContinuousInput(-Math.PI, Math.PI);

// // 4. Construct command to follow trajectory
// SwerveControllerCommand swerveControllerCommand = new
// SwerveControllerCommand(
// trajectory,
// swerveSubsystem::getPose,
// DriveConstants.kDriveKinematics,
// xController,
// yController,
// thetaController,
// swerveSubsystem::setModuleStates,
// swerveSubsystem);

// // 5. Add some init and wrap-up, and return everything
// return new SequentialCommandGroup(
// new InstantCommand(() ->
// swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
// swerveControllerCommand,
// new InstantCommand(() -> swerveSubsystem.()));
// }
