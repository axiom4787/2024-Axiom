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
    public void periodic() {
      bottomMotor.set(1);
    }
  //   String state = XboxController.get(); 
  //   public void handleStates(){
  //     switch(state) {
      
  //     Case 
  //     topMotor.setInverted(true); 
  //     bottomMotor.setInverted(false); 
  //     bottomMotor.set(1);
  //     topMotor.setInverted(false); 
  //     bottomMotor.setInverted(true); 
  //   }
  // }
  }