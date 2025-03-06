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
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkFlex leftMotor;
  private SparkFlex rightMotor;
  private SparkFlexConfig motorConfig;
  private final SparkClosedLoopController leftClosedLoopController;
  private final SparkClosedLoopController rightClosedLoopController;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  private DigitalInput resetlimitSwitch;
  private DigitalInput toplimitSwitch;

  private double currentSetPoint = 0;
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    leftMotor = new SparkFlex(Constants.ElevatorConstants.LEFT_CAN_ID, MotorType.kBrushless);
    rightMotor = new SparkFlex(Constants.ElevatorConstants.RIGHT_CAN_ID, MotorType.kBrushless);
    leftClosedLoopController = leftMotor.getClosedLoopController();
    rightClosedLoopController = rightMotor.getClosedLoopController();
    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();
    
    resetlimitSwitch = new DigitalInput(Constants.ElevatorConstants.RESET_LIMIT_PORT);
    toplimitSwitch = new DigitalInput(Constants.ElevatorConstants.TOP_LIMIT_PORT);

    motorConfig = new SparkFlexConfig();

        motorConfig.encoder
            .positionConversionFactor(0.00925)
            .velocityConversionFactor(1);

        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(1.0)
            .i(1e-4)
            .d(1)
            .outputRange(-1, 1)
            .p(0.0001, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        leftMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Initialize dashboard values
        SmartDashboard.setDefaultNumber("Elevator Target Position", 0);
        SmartDashboard.setDefaultNumber("Elevator Target Velocity", 0);
        resetEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator setPoint", currentSetPoint);
    SmartDashboard.putNumber("Elevator Left Encoder Position", leftEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Right Encoder Position", rightEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Left Velocity", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Elevator Right Velocity", rightEncoder.getVelocity());

    // Check the limit switch and reset the encoder if it is pressed
    if (isResetLimitSwitchPressed()) {
      resetEncoder();
    }
  }
  
  // Define the method only once
  public boolean isResetLimitSwitchPressed() {
    return !resetlimitSwitch.get();  // Assuming limit switch is normally closed
  }

  public boolean isTopLimitSwitchPressed() {
    return !toplimitSwitch.get(); // Assuming it's normally closed
  }

  public void resetEncoder(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }
  public void incrementPosition() {
    if (isTopLimitSwitchPressed()) {
      stop(); // Prevent further movement
      return;
  }
    currentSetPoint += Constants.ElevatorConstants.stepValue;
    goToPosition(currentSetPoint);
  }

  public void decrementPosition() {
    currentSetPoint -= Constants.ElevatorConstants.stepValue;
    goToPosition(currentSetPoint);
  }

  public void goToPosition(double value) {
    if (isTopLimitSwitchPressed() && value > getCurrentPosition()) {
      stop(); // Prevent moving beyond the top limit
      return;
  }
    currentSetPoint = value;
    leftClosedLoopController.setReference(value, ControlType.kPosition, ClosedLoopSlot.kSlot1);
    rightClosedLoopController.setReference(value, ControlType.kPosition, ClosedLoopSlot.kSlot1);
  }

  public double getCurrentPosition() {
    return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
  }

  public void stop() {
    leftMotor.set(0);
    rightMotor.set(0);
  }
}
