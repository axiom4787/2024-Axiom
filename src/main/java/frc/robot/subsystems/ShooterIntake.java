package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmIntakeShooter;

import java.util.concurrent.TimeUnit;

import java.util.concurrent.TimeUnit;

import com.revrobotics.CANSparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;


public class ShooterIntake extends SubsystemBase {
  private final CANSparkMax topMotor; //neo
  private final CANSparkMax bottomMotor; //neo 550

  private final RelativeEncoder topEncoder; //encoder for top 
  private final RelativeEncoder bottomEncoder; //encoder for bottom

  private String state = "off";

  XboxController driverXbox = new XboxController(0);

  public ShooterIntake() {
    topMotor = new CANSparkMax(Constants.ArmIntakeShooter.TOP_MOTORID, CANSparkMax.MotorType.kBrushless);
    bottomMotor = new CANSparkMax(Constants.ArmIntakeShooter.BOTTOM_MOTORID, CANSparkMax.MotorType.kBrushless);

    topMotor.setIdleMode(IdleMode.kBrake);

    topMotor.restoreFactoryDefaults();
    bottomMotor.restoreFactoryDefaults();

    topMotor.setInverted(true); 
    bottomMotor.setInverted(false); 

    // topMotor.follow(bottomMotor);

    topMotor.setSmartCurrentLimit(40);
    topMotor.setOpenLoopRampRate(0.05);
    bottomMotor.setSmartCurrentLimit(Constants.ArmIntakeShooter.BOTTOM_MOTOR_CURRENT_LIMIT);

    // topMotor.setIdleMode(TurretConstants.kShootMotorIdleMode);
    // bottomMotor.setIdleMode(TurretConstants.kRotateMotorIdleMode);

    topEncoder = topMotor.getEncoder();
    bottomEncoder = bottomMotor.getEncoder();
  }

    public void setState(String state) {
      this.state = state;
    }

    private void stateMachine() {
      if (state.equals("shoot")) {  // Use .equals for string comparison in Java
        if (driverXbox.getLeftTriggerAxis() > 0.9) {
          // Start bottom motor immediately
          topMotor.set(-1);
    
          // Create a new Thread to handle the delay and top motor activation
          new Thread(() -> {
            // Delay using FPGA Timer
            Timer.delay(Constants.ArmIntakeShooter.SHOOT_DELAY);  // Assuming SHOOT_DELAY is defined in your constants
    
            // After delay, start the top motor
            bottomMotor.set(-1);
          }).start();
    
        } else if (driverXbox.getRightTriggerAxis() > 0.9) {
          // Similar logic for the opposite condition, adjust motor speeds and directions as necessary
          topMotor.set(0.3);
          bottomMotor.set(0.3);
    
        } else {
          // Stop motors if neither condition is met
          bottomMotor.set(0.0);
          topMotor.set(0.0);
        }
      }
      
      else if (state.equals("intake")) {
        // Your intake logic here
          bottomMotor.set(0.0);
          topMotor.set(-1.0);
      }
      else if (state.equals("off")) {
        bottomMotor.set(0.0);
        topMotor.set(0.0);
      }
      else {
        // Any additional states or default behavior
      }
    }
    

    @Override
    public void periodic() {
      // simStateMachine(); 
      stateMachine(); 
      //System.out.println("motor output" + topMotor.getBusVoltage());
      
      // System.out.println(state);
      // topMotor.set(1);
    }
  
}
