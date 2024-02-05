package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmIntakeShooter;
import frc.robot.Constants.SetPointAngles;
import edu.wpi.first.wpilibj.XboxController;

public class ArmSubsystem extends SubsystemBase {
  public final CANSparkMax armLeftMotor; //neo
  private final CANSparkMax armRightMotor; //neo

  private final RelativeEncoder armLeftEncoder; //encoder for left arm
  private final RelativeEncoder armRightEncoder; //encoder for right arm

  private static double kP = 1.0;
  private static double kI = 0.0;
  private static double kD = 0.0;

  XboxController xboxcontroller = new XboxController(0);
  private PIDController ArmPID = new PIDController(kP, kI, kD);
    
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
  }

  public void CalculateArmPID(double kSetpoint) {
    armLeftMotor.setVoltage(MathUtil.clamp(ArmPID.calculate(armLeftEncoder.getPosition(), kSetpoint) + ArmIntakeShooter.FEED_FOWARD, 0, 12));
    ArmPID.reset();
  }

  // public void ArmMove(double Movement){
  //   armLeftMotor.set(Movement/2.5); //Change this for more motor power
  // }

    
  @Override
  public void periodic() {
  // This method will be called once per scheduler run
    if (xboxcontroller.getPOV() == 180) {
      CalculateArmPID(SetPointAngles.INTAKE_GROUND_ANGLE);
    }
    else if (xboxcontroller.getPOV() == 270) {
      CalculateArmPID(SetPointAngles.SHOOTER_AMP_ANGLE);
    }
    else if (xboxcontroller.getPOV() == 90) {
      CalculateArmPID(SetPointAngles.SHOOTER_SPEAKER_ANGLE);
    }
    else if (xboxcontroller.getPOV() == 0) {
      CalculateArmPID(SetPointAngles.INTAKE_HUMAN_ANGLE);
    }
    // need an else statement once triggers ready.
  }
}
