package frc.robot.subsystems;

//Gear ratios:
//NEO Vortex motor 60:1, Chain 1.857:1, Total 111.429:1

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ClimbSubsystem extends SubsystemBase {

  private SparkFlex motor;
  private SparkMax ratchetMotor;

  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;

  private double currentSetPoint = 0;
  private double ratchetMotorSpeed = 0.2;
  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    motor = new SparkFlex(Constants.ClimbConstants.CAN_ID, MotorType.kBrushless);
    ratchetMotor = new SparkMax(Constants.ClimbConstants.RATCHET_CAN_ID, MotorType.kBrushless);

    closedLoopController = motor.getClosedLoopController();
    
    encoder = motor.getEncoder();

    SparkFlexConfig globalConfig = new SparkFlexConfig();
    SparkMaxConfig ratchetConfig = new SparkMaxConfig();
    SparkFlexConfig motorConfig = new SparkFlexConfig();

    motorConfig.encoder
      .positionConversionFactor(1.0/324.0)
      .velocityConversionFactor(1.0/324.0);

    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);


    ratchetConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake);
    globalConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake);
            
    motorConfig.apply(globalConfig);
    ratchetConfig.apply(ratchetConfig);
    
    ratchetMotor.configure(ratchetConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Climb Target Position", 0);
    SmartDashboard.setDefaultNumber("Climb Target Velocity", 0);

    resetEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climb setPoint", currentSetPoint);
    SmartDashboard.putNumber("Climb Encoder Position", getCurrentPosition());
    SmartDashboard.putNumber("Climb Velocity", encoder.getVelocity());
  }
  
  public void runRatchet(){
   ratchetMotor.set(ratchetMotorSpeed);
  }
  public void stopRatchet(){
    ratchetMotor.set(0.0);
  }

  public void resetEncoder(){
    encoder.setPosition(0);
  }
  public void incrementPosition() {
    currentSetPoint += Constants.ClimbConstants.stepValue;
    goToPosition(currentSetPoint);
  }

  public void decrementPosition() {
    currentSetPoint -= Constants.ClimbConstants.stepValue;
    goToPosition(currentSetPoint);
  }

  public void goToPosition(double value) {
    currentSetPoint = value;
    closedLoopController.setReference(value, ControlType.kPosition, ClosedLoopSlot.kSlot1);
  }

  public double getCurrentPosition() {
    return encoder.getPosition();
  }

  public void stop() {
    var speed = 0;
    motor.set(speed);
    ratchetMotor.set(speed);
  }
}