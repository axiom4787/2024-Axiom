package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmIntakeShooter;

import java.util.concurrent.TimeUnit;

import java.util.concurrent.TimeUnit;

import com.revrobotics.CANSparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;


public class ShooterIntake extends SubsystemBase {
    
  private final CANSparkMax topMotor; //neo
  private final CANSparkMax bottomMotor; //neo 550

  private final RelativeEncoder topEncoder; //encoder for top 
  private final RelativeEncoder bottomEncoder; //encoder for bottom

  public String state = "";

  XboxController driverXbox = new XboxController(0);
  public ShooterIntake() {

    topMotor = new CANSparkMax(Constants.ArmIntakeShooter.TOP_MOTORID, CANSparkMax.MotorType.kBrushless);
    bottomMotor = new CANSparkMax(Constants.ArmIntakeShooter.BOTTOM_MOTORID, CANSparkMax.MotorType.kBrushless);

    topMotor.restoreFactoryDefaults();
    bottomMotor.restoreFactoryDefaults();

    topMotor.setInverted(false); 
    bottomMotor.setInverted(true); 

    topMotor.follow(bottomMotor);

    topMotor.setSmartCurrentLimit(Constants.ArmIntakeShooter.TOP_MOTOR_CURRENT_LIMIT);
    bottomMotor.setSmartCurrentLimit(Constants.ArmIntakeShooter.BOTTOM_MOTOR_CURRENT_LIMIT);

    // topMotor.setIdleMode(TurretConstants.kShootMotorIdleMode);
    // bottomMotor.setIdleMode(TurretConstants.kRotateMotorIdleMode);


    topEncoder = topMotor.getEncoder();
    bottomEncoder = bottomMotor.getEncoder();
  }
  public void setIntake() {
    state = "intake";
  }
  public void setShoot() {
    state = "shoot";
  }
  public void setShooterIntake() {
    switch(state) {
      case "intake":
        topMotor.setInverted(false); 
        bottomMotor.setInverted(true); 
        bottomMotor.set(1.0);
        TimeUnit.SECONDS.sleep(2); // want to make sure that the motor spins for long enough before it stops moving
        break;
      case "shoot":
        topMotor.setInverted(true); 
        bottomMotor.setInverted(false); 
        bottomMotor.set(1.0);
        TimeUnit.SECONDS.sleep(3); // want to make sure that the motor spins for long enough before it stops moving
        break;
      default:
        bottomMotor.set(0.0);
      }
  }
  public void SimSetShooterIntake() {
    switch(state) {
      case "intake":
        topMotor.setInverted(false); 
        bottomMotor.setInverted(true);
        bottomMotor.setVoltage(1.0);
        wait(300); // want to make sure that the motor spins for long enough before it stops moving
        break;
      case "shoot":
        topMotor.setInverted(true); 
        bottomMotor.setInverted(false); 
        bottomMotor.setVoltage(1.0);
        wait(300); // want to make sure that the motor spins for long enough before it stops moving
        break;
      default:
        bottomMotor.set(0.0);
      }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler
    
    }
}
