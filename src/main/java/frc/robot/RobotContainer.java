package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.Board.DriverTab;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {

        private SwerveSubsystem swerveSubsystem;
        
        CommandXboxController m_driverController;
        private RobotShared m_robotShared = RobotShared.getInstance();

        SendableChooser<Command> autoChooser;

        public RobotContainer() {
                initSubsystems();
                initInputDevices();
                NamedCommands.registerCommand("ResetGyro", new InstantCommand(() -> swerveSubsystem.zeroHeading()));

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Mode", autoChooser);

                configureButtonBindings();
        }

        private void initSubsystems() {
                m_robotShared = RobotShared.getInstance();

                swerveSubsystem = m_robotShared.getDriveSubsystem();
                // m_robotShared.getPhotonVision();

                DriverTab.getInstance();
        }
        private void initInputDevices(){
                m_driverController = m_robotShared.getDriverController();
        }

        private void configureButtonBindings() {
                m_driverController.a().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

                m_driverController.leftStick()
                                .whileTrue(new RunCommand(
                                                () -> swerveSubsystem.drive(
                                                                -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                                                OIConstants.kDeadband) / 2,
                                                                -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                                                OIConstants.kDeadband) / 2,
                                                                -MathUtil.applyDeadband(m_driverController.getRightX(),
                                                                                OIConstants.kDeadband) / 2,
                                                                true, true, OIConstants.kUseQuadraticInput),
                                                swerveSubsystem));

                m_driverController.b().onTrue(new InstantCommand(() -> swerveSubsystem.setForwardFormation()));
                m_driverController.x().onTrue(new InstantCommand(() -> swerveSubsystem.setXFormation()));
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
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
