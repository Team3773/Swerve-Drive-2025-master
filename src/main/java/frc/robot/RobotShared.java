package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotShared {
  private static RobotShared instance;
  protected SwerveSubsystem m_robotDrive = null;

  protected final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  public static RobotShared getInstance() {
    if (instance == null) {
      instance = new RobotShared();
    }
    return instance;
  }

  Optional<Alliance> m_alliance = DriverStation.getAlliance();

  public Alliance getAlliance() { // blue is default for the path planner (paths are made on the blue side)

    if (m_alliance.isPresent()) {
      if (m_alliance.get() == Alliance.Blue) {
        return Alliance.Blue;
      } else {
        return Alliance.Red;
      }
    } else {
      System.err.println("No alliance found!");
      return Alliance.Blue;
    }
  }

  public SwerveSubsystem getDriveSubsystem() {
    if(m_robotDrive == null) {
      m_robotDrive = new SwerveSubsystem();
    }
    return m_robotDrive;
  }

  public CommandXboxController getDriverController() {
    return m_driverController;
  }
}
