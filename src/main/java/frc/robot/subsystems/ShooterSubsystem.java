package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.DigitalInput;
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

//NEO 550 with Spark MAX and 3:1 Gear Ratio

/** Creates new ShooterSubsystem. */
public class ShooterSubsystem extends SubsystemBase {
    
  private SparkMax motor;
  private SparkMaxConfig motSparkMaxConfig;
  private DigitalInput beamBreakSensor;

    public ShooterSubsystem() {
        motor = new SparkMax(Constants.ShooterConstants.LEFT_CAN_ID, MotorType.kBrushless);
        motor = new SparkMax(Constants.ShooterConstants.RIGHT_CAN_ID, MotorType.kBrushless);

         // Initialize the DigitalInput object for the beam break sensor (replace 0 with your digital port number)
         beamBreakSensor = new DigitalInput(0);}

         public void checkBeamAndControlMotor() {
         // Read the value from the sensor
         boolean beamBroken = !beamBreakSensor.get();
 
         // Check if the beam is broken
         if (beamBroken) {
             // Stop the motor if the beam is broken
             motor.set(0.0);
             System.out.println("Beam broken! Motor stopped.");
         } else {
             // Keep the motor running (e.g., at half speed) if the beam is intact
             motor.set(0.5);
             System.out.println("Beam intact! Motor running.");
         }
    }
}
