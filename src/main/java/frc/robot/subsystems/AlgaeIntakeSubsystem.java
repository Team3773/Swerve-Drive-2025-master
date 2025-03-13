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

public class AlgaeIntakeSubsystem extends SubsystemBase {
    
    private SparkMax motor;
   
    /** Creates new ShooterSubsystem. */
             public AlgaeIntakeSubsystem() {
              motor = new SparkMax(Constants.AlgaeIntakeSubystemConstants.CAN_ID, MotorType.kBrushless);
             }
      
              public void startIntake() {
                motor.set(0.5);
                System.out.println("Motor running forward.");
            }
        
            public void reverseIntake() {
                motor.set(-0.5);
                System.out.println("Motor running in reverse.");
            }
        
            public void stopIntake() {
                motor.set(0.0);
                System.out.println("Motor stopped.");
            }

            public void runMotors(double speed) {
                motor.set(speed);
            }
        }
            
        
        