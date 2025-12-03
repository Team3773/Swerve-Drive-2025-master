// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    private SparkMax leftMotor;
    private SparkMax rightMotor;
    private CANcoder CANcoder;
    private final SparkClosedLoopController leftClosedLoopController;
    // private final SparkClosedLoopController rightClosedLoopController;
    // private final RelativeEncoder absoluteEncoder;
    // private final SparkAbsoluteEncoder absoluteEncoder;
    // private final RelativeEncoder rightEncoder;
    private DigitalInput resetlimitSwitch;
    // private DigitalInput toplimitSwitch;

    private double currentSetPoint = 0;

    /** Creates a new ElevatorSubsystem. */
    public ElevatorSubsystem() {  
        leftMotor = new SparkMax(Constants.ElevatorConstants.LEFT_CAN_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.ElevatorConstants.RIGHT_CAN_ID, MotorType.kBrushless);
        CANcoder = new CANcoder(Constants.ElevatorConstants.CANcoder_ID);
        
        // rightClosedLoopController = rightMotor.getClosedLoopController();
        // leftEncoder = leftMotor.getEncoder();
        
        // rightEncoder = rightMotor.getEncoder();

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        resetlimitSwitch = new DigitalInput(Constants.ElevatorConstants.RESET_LIMIT_PORT);
        // toplimitSwitch = new DigitalInput(Constants.ElevatorConstants.TOP_LIMIT_PORT);

        leaderConfig.absoluteEncoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1)
                .zeroOffset(Constants.ElevatorConstants.AbsoluteEncoderOffset);

        leaderConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                .p(1.0)
                .i(1e-4)
                .d(1)
                .outputRange(-1, 1)
                .p(0.1, ClosedLoopSlot.kSlot1)
                .i(0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        globalConfig
                .smartCurrentLimit(50)
                .idleMode(IdleMode.kBrake);

        leaderConfig.apply(globalConfig);
        followerConfig.apply(globalConfig).follow(leftMotor);

        leftMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rightMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Initialize dashboard values
        SmartDashboard.setDefaultNumber("Elevator Target Position", 0);
        SmartDashboard.setDefaultNumber("Elevator Target Velocity", 0);
        
        //Assume position is below the limit switch on startup
        // leftEncoder.setPosition(0);
        leftClosedLoopController = leftMotor.getClosedLoopController();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Elevator setPoint", currentSetPoint);
        SmartDashboard.putNumber("Elevator Left Encoder Position", getPostion());
        // SmartDashboard.putNumber("Elevator Right Encoder Position",
        // rightEncoder.getPosition());
        SmartDashboard.putNumber("Elevator Left Velocity", getPostion());
        // SmartDashboard.putNumber("Elevator Right Velocity",
        // rightEncoder.getVelocity());

        // Check the limit switch and reset the encoder if it is pressed
        if (isResetLimitSwitchPressed()) {
            // resetEncoder();
            if(this.currentSetPoint <= this.getPostion()){
              this.stop();
          }
        }
    }

    // Define the method only once
    public boolean isResetLimitSwitchPressed() {
        return resetlimitSwitch.get(); // Assuming limit switch is normally closed
    }

    // public boolean isTopLimitSwitchPressed() {
    //     return toplimitSwitch.get(); // Assuming it's normally closed
    // }

    // public void resetEncoder() {
    //     leftEncoder.setPosition(0);
    // }

    public void incrementPosition() {
        // if (isTopLimitSwitchPressed()) {
        //     stop(); // Prevent further movement
        //     return;
        // }
        currentSetPoint += Constants.ElevatorConstants.stepValue;
        goToPosition(currentSetPoint);
    }

    public void decrementPosition() {
        currentSetPoint -= Constants.ElevatorConstants.stepValue;
        goToPosition(currentSetPoint);
    }

    public void goToPosition(double value) {
        currentSetPoint = value;
        leftClosedLoopController.setReference(value, ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }

    public double getPostion(){
        return CANcoder.getAbsolutePosition().getValueAsDouble();
    }

    public void stop() {
        leftMotor.set(0);
    }
}
