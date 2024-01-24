package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ArmIntakeShooter;;

public class ArmSubsystem {
    private final CANSparkMax armLeftMotor; //neo
    private final CANSparkMax armRightMotor; //neo

    private final RelativeEncoder armLeftEncoder; //encoder for left arm
    private final RelativeEncoder armRightEncoder; //encoder for right arm

    public ArmSubsystem() {
      armLeftMotor = new CANSparkMax(ArmIntakeShooter.LEFTARM_MOTORID, CANSparkMax.MotorType.kBrushless);
      armRightMotor = new CANSparkMax(ArmIntakeShooter.RIGHTARM_MOTORID, CANSparkMax.MotorType.kBrushless);

      armLeftMotor.restoreFactoryDefaults();
      armRightMotor.restoreFactoryDefaults();

      armLeftMotor.setInverted(true);
      armRightMotor.setInverted(false);

      armRightMotor.follow(armLeftMotor);

      armLeftMotor.setSmartCurrentLimit(ArmIntakeShooter.LEFTARM_MOTOR_CURRENT_LIMIT);
      armRightMotor.setSmartCurrentLimit(ArmIntakeShooter.RIGHTARM_MOTOR_CURRENT_LIMIT);

      // armLeftMotor.setIdleMode(TurretArmIntakeShooter.kIntakeMotorIdleMode);
      // armRightMotor.setIdleMode(TurretArmIntakeShooter.kIntakeMotorIdleMode);

      armLeftEncoder = armLeftMotor.getEncoder();
      armRightEncoder = armRightMotor.getEncoder();
      
      // PIDController pid = new PIDController(ArmIntakeShooter.ShootingP, 
      //                                             ArmIntakeShooter.ShootingI, 
      //                                             ArmIntakeShooter.ShootingD); 
      // PIDController.enableContinuousOutput(-180, 180); 
      

      // armLeftMotor.set(pid.calculate(armLeftEncoder.getDistance(), setPoint) + feedforward);
      // armRightMotor.set(pid.calculate(armRightEncoder.getDistance(), setPoint) + feedforward);
    }
}
