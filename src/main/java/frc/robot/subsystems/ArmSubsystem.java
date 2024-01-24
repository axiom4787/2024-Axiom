package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmIntakeShooter;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax armLeftMotor; //neo
    private final CANSparkMax armRightMotor; //neo

    private final RelativeEncoder armLeftEncoder; //encoder for left arm
    private final RelativeEncoder armRightEncoder; //encoder for right arm

    private int currentPointNum = 0; 
    private static int kP;
    private static int kI;
    private static int kD;

    PIDController ArmPID = new PIDController(kP, kI, kD);
    
    public ArmSubsystem() {

      ArmPID.enableContinuousInput(-180, 180);

      armLeftMotor = new CANSparkMax(ArmIntakeShooter.LEFTARM_MOTORID, CANSparkMax.MotorType.kBrushless);
      armRightMotor = new CANSparkMax(ArmIntakeShooter.RIGHTARM_MOTORID, CANSparkMax.MotorType.kBrushless);

      armLeftMotor.restoreFactoryDefaults();
      armRightMotor.restoreFactoryDefaults();

      armLeftMotor.setInverted(true);
      armRightMotor.setInverted(false);

      armRightMotor.follow(armLeftMotor);

      armLeftMotor.setSmartCurrentLimit(ArmIntakeShooter.LEFTARM_MOTOR_CURRENT_LIMIT);
      armRightMotor.setSmartCurrentLimit(ArmIntakeShooter.RIGHTARM_MOTOR_CURRENT_LIMIT);

      armLeftMotor.setIdleMode(IdleMode.kBrake);
      armRightMotor.setIdleMode(IdleMode.kBrake);

      armLeftEncoder = armLeftMotor.getEncoder();
      armRightEncoder = armRightMotor.getEncoder();
      
      // PIDController pid = new PIDController(ArmIntakeShooter.ShootingP, 
      //                                             ArmIntakeShooter.ShootingI, 
      //                                             ArmIntakeShooter.ShootingD); 
      // PIDController.enableContinuousOutput(-180, 180); 
      

      // armLeftMotor.set(pid.calculate(armLeftEncoder.getDistance(), setPoint) + feedforward);
      // armRightMotor.set(pid.calculate(armRightEncoder.getDistance(), setPoint) + feedforward);
    }
    public void ArmPID(double goalPoint, int newPointNum) {
      if (currentPointNum != newPointNum){
        currentPointNum = newPointNum;
        ArmPID = new PIDController(kP, kI, kD);
        ArmPID.setSetpoint(armLeftEncoder.getPosition());
      }
      if (goalPoint == 0 && (armLeftEncoder.getPosition() < 8)) {
        armLeftMotor.set(MathUtil.clamp(ArmPID.calculate(armLeftEncoder.getPosition(), goalPoint), -0.075, .4));
      } else {
        armLeftMotor.set(MathUtil.clamp(ArmPID.calculate(armLeftEncoder.getPosition(), goalPoint), -0.075, .4));
      } 
  }

  public void ArmMove(double Movement){
    armLeftMotor.set(Movement/2.5); //Change this for more motor power
  }
}
