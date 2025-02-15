package frc.robot.subsystems;

//Gear ratios:
//NEO Vortex motor 60:1, Chain 1.857:1, Total 111.429:1

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ClimbSubsystem extends SubsystemBase {
    
    private SparkFlex motor;
    private SparkFlexConfig motorConfig;
    private RelativeEncoder encoder;

    double currentRotationSetpoint = 0;
    double armRotationStepValue = 0.01;

    /*Creates the ClimbSubsystem */
    public ClimbSubsystem() {
        motor = new SparkFlex(Constants.ClimbConstants.CAN_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        motorConfig = new SparkFlexConfig();

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
        SmartDashboard.putNumber("Climb setPoint", currentRotationSetpoint);
        SmartDashboard.putNumber("Climb Encoder Position", encoder.getPosition());
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
        System.out.println("Climb speed:" + speed);
        motor.set(speed);
      }
}
