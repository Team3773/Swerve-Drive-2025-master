package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//NEO 550 with Spark MAX and 5:1 Gear Ratio

public class ShooterSubsystem extends SubsystemBase {

  private SparkMax leftMotor;
  private SparkMax rightMotor;
  private AnalogInput receiverInput;
  /** Creates new ShooterSubsystem. */
    public ShooterSubsystem() {
        leftMotor = new SparkMax(Constants.ShooterConstants.LEFT_CAN_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.ShooterConstants.RIGHT_CAN_ID, MotorType.kBrushless);

         // Initialize the DigitalInput object for the beam break sensor (replace 0 with your digital port number)
         receiverInput =  new AnalogInput(Constants.ShooterConstants.RECEIVER_PORT);
        
        }

         public void checkBeamAndControlMotor(boolean rightBumperPressed, boolean rightTriggerPressed, boolean SpeedIncrease) {
         // Read the value from the sensor
        
         double leftmotorSpeed = 0.5;
         double rightmotorSpeed = 0.5;
         if(SpeedIncrease){
            rightmotorSpeed = rightmotorSpeed * 1.5;
         }
         if (rightBumperPressed) {
            // Always run the motors when the right bumper is pressed
            leftMotor.set(-leftmotorSpeed);
            rightMotor.set(rightmotorSpeed);
            System.out.println("Right bumper pressed! Motor running regardless of beam state.");
        } else if (rightTriggerPressed) {
            // Run motors when right trigger is pressed and beam is not broken
            if (!beamBroken()) {
                leftMotor.set(-leftmotorSpeed);
                rightMotor.set(rightmotorSpeed);
                System.out.println("Right trigger pressed and beam intact! Motor running.");
            } else {
                // Stop motors if the beam is broken
                leftMotor.set(0.0);
                rightMotor.set(0.0);
                System.out.println("Right trigger pressed but beam broken! Motor stopped.");
            }
        } else {
            // Stop motors if neither the right bumper nor the right trigger is pressed
            leftMotor.set(0.0);
            rightMotor.set(0.0);
            System.out.println("Neither right bumper nor right trigger pressed! Motor stopped.");
        }

         
         System.out.println("Beam Break State "+ receiverInput.getValue());

    }

    public boolean beamBroken(){
        return receiverInput.getValue() < 50;
    }

    public void runMotors(double speed) {
        // Adjust speed based on gear ratio
        double adjustedSpeed = speed;
        leftMotor.set(adjustedSpeed);
        rightMotor.set(adjustedSpeed);
    }
}