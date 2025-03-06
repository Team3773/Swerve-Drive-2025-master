// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkFlex LeadMotor;
  private SparkFlex FollowerMotor;
  
  private final RelativeEncoder LeadEncoder;
  private DigitalInput resetlimitSwitch;
  private DigitalInput toplimitSwitch;
  private final SparkFlexConfig leadSparkFlexConfig;
  private final SparkFlexConfig followerSparkFlexConfig;
  private final PIDController pidController;

  private double currentElevatorPosition;
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    LeadMotor = new SparkFlex(Constants.ElevatorConstants.LEFT_CAN_ID, MotorType.kBrushless);
    FollowerMotor = new SparkFlex(Constants.ElevatorConstants.RIGHT_CAN_ID, MotorType.kBrushless);
    LeadEncoder = LeadMotor.getEncoder();
    
    resetlimitSwitch = new DigitalInput(Constants.ElevatorConstants.RESET_LIMIT_PORT);
    toplimitSwitch = new DigitalInput(Constants.ElevatorConstants.TOP_LIMIT_PORT);

    pidController = new PIDController(ElevatorConstants.kElevator_P, 
                                      ElevatorConstants.kElevator_I, 
                                      ElevatorConstants.kElevator_D);

    leadSparkFlexConfig = new SparkFlexConfig();
    followerSparkFlexConfig = new SparkFlexConfig();

        leadSparkFlexConfig.idleMode(IdleMode.kCoast);
        leadSparkFlexConfig.inverted(false);
        LeadMotor.configure(leadSparkFlexConfig, null, null);
       
        followerSparkFlexConfig.idleMode(IdleMode.kCoast);
        followerSparkFlexConfig.follow(LeadMotor, false);
        FollowerMotor.configure(followerSparkFlexConfig, null, null);
        
        LeadEncoder.setPosition(0); 
  }

   // getters
   public double getElevatorPosition()
   {
       return LeadEncoder.getPosition();
   }

   public double getSpeed()
   {
       return LeadMotor.get();
   }

   public Boolean getUpperLimitState()
   {
       return toplimitSwitch.get();
   }

   public Boolean getLowerLimitState()
   {
       return resetlimitSwitch.get();
   }

   /* 
   public Boolean limitStateCheck(){
       if (!getUpperLimitState() && !getLowerLimitState()){
           return true;
       }
       else return false;
   }
   */
   // Setters
   public void setSpeed(double speed, double speedCorrectionCoef)
   {
       LeadMotor.set(speed);
   }

   public void setElevatorPosition(double targetPosition, double correctionCoef)
   {
       currentElevatorPosition = getElevatorPosition();
       setSpeed(pidController.calculate(currentElevatorPosition, targetPosition), correctionCoef); // Integrate the PID controller her
   }

   public void stop()
   {
        LeadMotor.set(0);
   }

   public void resetEncoder()
   {
       LeadEncoder.setPosition(0);
   }

   @Override
 public void periodic() { 
 }

}
