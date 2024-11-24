package frc.Board;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class CurrentDrawTab {
  ShuffleboardTab m_sbt_CurrentDrawTab;

  GenericEntry m_nte_DrivingFrontLeft;
  GenericEntry m_nte_DrivingFrontRight;
  GenericEntry m_nte_DrivingRearLeft;
  GenericEntry m_nte_DrivingRearRight;

  GenericEntry m_nte_TurningFrontLeft;
  GenericEntry m_nte_TurningFrontRight;
  GenericEntry m_nte_TurningRearLeft;
  GenericEntry m_nte_TurningRearRight;

  private static CurrentDrawTab instance = null;

  private CurrentDrawTab() {
    initShuffleboardTab();
  }

  public static CurrentDrawTab getInstance() {
    if(instance == null) {
      instance = new CurrentDrawTab();
    }
    return instance;
  }

  private void initShuffleboardTab() {
    // Create and get reference to SB tab
    m_sbt_CurrentDrawTab = Shuffleboard.getTab("CurrentDraw");


    m_nte_DrivingFrontLeft = m_sbt_CurrentDrawTab.add("DrivingFrontLeft", 0.0)
      .withSize(2, 1).getEntry();
    
    m_nte_DrivingFrontRight = m_sbt_CurrentDrawTab.add("DrivingFrontRight", 0.0)
      .withSize(2, 1).getEntry();

    m_nte_DrivingRearLeft = m_sbt_CurrentDrawTab.add("DrivingRearLeft", 0.0) 
      .withSize(2, 1).getEntry();
    
    m_nte_DrivingRearRight = m_sbt_CurrentDrawTab.add("DrivingRearRight", 0.0)
      .withSize(2, 1).getEntry();
    
    m_nte_TurningFrontLeft = m_sbt_CurrentDrawTab.add("TurningFrontLeft", 0.0)
      .withSize(2, 1).getEntry();
    
    m_nte_TurningFrontRight = m_sbt_CurrentDrawTab.add("TurningFrontRight", 0.0) 
      .withSize(2, 1).getEntry();
    
    m_nte_TurningRearLeft = m_sbt_CurrentDrawTab.add("TurningRearLeft", 0.0)
      .withSize(2, 1).getEntry();
    
    m_nte_TurningRearRight = m_sbt_CurrentDrawTab.add("TurningRearRight", 0.0)
      .withSize(2, 1).getEntry();
  }
  
  public void setDrivingFrontLeft(double currentDraw) {
    m_nte_DrivingFrontLeft.setDouble(currentDraw);
  }
  public void setDrivingFrontRight(double currentDraw) {
    m_nte_DrivingFrontRight.setDouble(currentDraw);
  }
  public void setDrivingRearLeft(double currentDraw) {
    m_nte_DrivingRearLeft.setDouble(currentDraw);
  }
  public void setDrivingRearRight(double currentDraw) {
    m_nte_DrivingRearRight.setDouble(currentDraw);
  }
  public void setTurningFrontLeft(double currentDraw) {
    m_nte_TurningFrontLeft.setDouble(currentDraw);
  }
  public void setTurningFrontRight(double currentDraw) {
    m_nte_TurningFrontRight.setDouble(currentDraw);
  }
  public void setTurningRearLeft(double currentDraw) {
    m_nte_TurningRearLeft.setDouble(currentDraw);
  }
  public void setTurningRearRight(double currentDraw) {
    m_nte_TurningRearRight.setDouble(currentDraw);
  }
}
