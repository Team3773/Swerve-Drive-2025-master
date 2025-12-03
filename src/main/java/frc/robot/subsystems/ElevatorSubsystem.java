// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    // Limits
    private static double kDt = 0.02;
    private static double kMaxVelocity = 1.75;
    private static double kMaxAcceleration = 0.75;
    private static double kP = 1.3;
    private static double kI = 0.0;
    private static double kD = 0.7;
    private static double kS = 1.1;
    private static double kG = 1.2;
    private static double kV = 1.3;

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxVelocity,
            kMaxAcceleration);
    private final ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV);

    private SparkMax leftMotor;
    private SparkMax rightMotor;
    private CANcoder CANcoder;
    private DigitalInput resetlimitSwitch;
    private double currentSetPoint = 0;

    /** Creates a new ElevatorSubsystem. */
    public ElevatorSubsystem() {
        leftMotor = new SparkMax(Constants.ElevatorConstants.LEFT_CAN_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.ElevatorConstants.RIGHT_CAN_ID, MotorType.kBrushless);
        CANcoder = new CANcoder(Constants.ElevatorConstants.CANcoder_ID);

        var toApply = new CANcoderConfiguration();
        /*
         * User can change the configs if they want, or leave it empty for
         * factory-default
         */
        CANcoder.getConfigurator().apply(toApply);
        /* Speed up signals to an appropriate rate */
        BaseStatusSignal.setUpdateFrequencyForAll(100, CANcoder.getPosition(), CANcoder.getVelocity());

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        resetlimitSwitch = new DigitalInput(Constants.ElevatorConstants.RESET_LIMIT_PORT);

        globalConfig
                .smartCurrentLimit(50)
                .idleMode(IdleMode.kBrake);

        leaderConfig.apply(globalConfig);
        followerConfig.apply(globalConfig).follow(leftMotor);

        leftMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rightMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Initialize dashboard values
        SmartDashboard.setDefaultNumber("Elevator setPoint", 0);
        SmartDashboard.setDefaultNumber("Elevator Encoder Position", -1);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Elevator setPoint", currentSetPoint);
        SmartDashboard.putNumber("Elevator Encoder Position", getPostion());

        // Check the limit switch and reset the encoder if it is pressed
        if (isResetLimitSwitchPressed()) {
             resetEncoder();
             goToPosition(0);
        }
    }

    // Define the method only once
    public boolean isResetLimitSwitchPressed() {
        return resetlimitSwitch.get(); // Assuming limit switch is normally closed
    }

    // public boolean isTopLimitSwitchPressed() {
    // return toplimitSwitch.get(); // Assuming it's normally closed
    // }

    public void resetEncoder() {
        CANcoder.setPosition(0);
    }

    public void incrementPosition() {
        // if (isTopLimitSwitchPressed()) {
        // stop(); // Prevent further movement
        // return;
        // }
        currentSetPoint += Constants.ElevatorConstants.stepValue;
        goToPosition(currentSetPoint);
    }

    public void decrementPosition() {
        currentSetPoint -= Constants.ElevatorConstants.stepValue;
        goToPosition(currentSetPoint);
    }

    public void goToPosition(double value) {

        pidController.setGoal(value);
        currentSetPoint = value;
        leftMotor.setVoltage(
                pidController.calculate(getPostion())
                        + m_feedforward.calculate(pidController.getSetpoint().velocity));
    }

    public double getPostion() {
        return CANcoder.getPosition().getValueAsDouble();
    }

    public void stop() {
        leftMotor.set(0);
    }
}
