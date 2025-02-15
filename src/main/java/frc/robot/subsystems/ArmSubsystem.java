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

  double currentRotationSetpoint = 0;
  double armRotationStepValue = 0.01;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    motor = new SparkMax(Constants.ArmConstants.CAN_ID, MotorType.kBrushless);
    encoder = motor.getEncoder();

    motorConfig = new SparkMaxConfig();

    motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control.
        .p(0)
        .i(0)
        .d(0)
        .outputRange(-0.25, 0.25)
        // Set PID values for velocity control.
        .velocityFF(0);
  }

      @Override
      public void periodic() {
        SmartDashboard.putNumber("Arm setPoint", currentRotationSetpoint);
        SmartDashboard.putNumber("Arm Encoder Position", encoder.getPosition());
      }
      
      public void incrementPosition(){
        currentRotationSetpoint += armRotationStepValue;
      }
      public void decrementPosition(){
        currentRotationSetpoint -= armRotationStepValue;
      }
    
      public void goToPosition(double value){
        currentRotationSetpoint = value;
      }
    
      public double getCurrentPosition(){
        return encoder.getPosition();
      }

      public void stopLift() {
        var speed = 0;
        System.out.println("Arm speed:" + speed);
        motor.set(speed);
      }
  }

