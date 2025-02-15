package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//NEO Motor Spark MAX with 67:1 Gear Ratio
//True Gear Ratio is currently 60:1 Gear Ratio * ~2:1 Sprocket Ratio = ~120:1 Ratio
public class ArmSubsystem extends SubsystemBase {

  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;

  private double currentSetPoint = 0;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    motor = new SparkMax(Constants.ArmConstants.CAN_ID, MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    motorConfig = new SparkMaxConfig();

    motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Arm Target Position", 0);
    SmartDashboard.setDefaultNumber("Arm Target Velocity", 0);

    resetEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm setPoint", currentSetPoint);
    SmartDashboard.putNumber("Arm Encoder Position", getCurrentPosition());
    SmartDashboard.putNumber("Arm Velocity", encoder.getVelocity());
  }

  public void resetEncoder(){
    encoder.setPosition(0);
  }
  public void incrementPosition() {
    currentSetPoint += Constants.ArmConstants.stepValue;
    goToPosition(currentSetPoint);
  }

  public void decrementPosition() {
    currentSetPoint -= Constants.ArmConstants.stepValue;
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
  }
}
