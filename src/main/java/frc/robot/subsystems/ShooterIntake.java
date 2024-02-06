package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmIntakeShooter;

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
  
    public String state = "";
    if (driverXbox.getAButton()) {
      state = "intake";
    }
    if (driverXbox.getYButton()) {
      state = "shoot";
    }

    if (driverXbox.getAButtonPressed() || driverXBox.getYButtonPressed()) {
      state = "off";
    }
    switch(state) {
    case "intake":
      topMotor.setInverted(false); 
      bottomMotor.setInverted(true); 
      bottomMotor.set(1.0);
      break;
    case "shoot":
      topMotor.setInverted(true); 
      bottomMotor.setInverted(false); 
      bottomMotor.set(1.0);
      break;
    case "off":
      bottomMotor.set(0.0);
    }
  }
