package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

//NEO 550 with Spark MAX and 5:1 Gear Ratio

public class ShooterSubsystem extends SubsystemBase {

  private SparkMax leftMotor;
  private SparkMax rightMotor;
  private DigitalInput receiverInput;
  /** Creates new ShooterSubsystem. */
    public ShooterSubsystem() {
        leftMotor = new SparkMax(Constants.ShooterConstants.LEFT_CAN_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.ShooterConstants.RIGHT_CAN_ID, MotorType.kBrushless);

         // Initialize the DigitalInput object for the beam break sensor (replace 0 with your digital port number)
         receiverInput = new DigitalInput(Constants.ShooterConstants.RECEIVER_PORT);}

         public void checkBeamAndControlMotor(boolean rightBumperPressed) {
         // Read the value from the sensor
         boolean receiverActive = receiverInput.get();
         double motorSpeed = 0.5;

         // Check if the beam is broken
         if (rightBumperPressed) {
             // Stop the motor if the beam is broken
             leftMotor.set(motorSpeed);
             rightMotor.set(motorSpeed);
             System.out.println("Right bumper pressed! Motor running.");
         } else if (receiverActive) {
             // Run the motors if the right bumper is pressed
             leftMotor.set(0.5);
             rightMotor.set(0.5);
             System.out.println("Beam intact! Motor running.");
         } else {
             // Stop the motor if the right bumper is not pressed
             leftMotor.set(0.0);
             rightMotor.set(0.0);
             System.out.println("Right bumper not pressed! Motor stopped.");
         }
    }

    public void runMotors(double speed) {
        // Adjust speed based on gear ratio
        double adjustedSpeed = speed;
        leftMotor.set(adjustedSpeed);
        rightMotor.set(adjustedSpeed);
    }
}
