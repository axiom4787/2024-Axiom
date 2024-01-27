package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmIntakeShooter;
import frc.robot.Constants.SetPointAngles;
import frc.robot.XBoxController;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax armLeftMotor; //neo
  private final CANSparkMax armRightMotor; //neo

  private final RelativeEncoder armLeftEncoder; //encoder for left arm
  private final RelativeEncoder armRightEncoder; //encoder for right arm

  private static int kP;
  private static int kI;
  private static int kD;

  private static XBoxController xboxcontroller = new XBoxController();


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
    armLeftMotor.set(ArmPID.calculate(armLeftEncoder.getPosition(), kSetpoint) + ArmIntakeShooter.FEED_FOWARD);
    ArmPID.reset();
  }

  public void ArmMove(double Movement){
    armLeftMotor.set(Movement/2.5); //Change this for more motor power
  }

    
  @Override
  public void periodic() {
  // This method will be called once per scheduler run
    if (xboxcontroller.getXboxDpad() == 180) {
      CalculateArmPID(SetPointAngles.INTAKE_GROUND_ANGLE);
    }
    else if (xboxcontroller.getXboxDpad() == 270) {
      CalculateArmPID(SetPointAngles.SHOOTER_AMP_ANGLE);
    }
    else if (xboxcontroller.getXboxDpad() == 90) {
      CalculateArmPID(SetPointAngles.SHOOTER_SPEAKER_ANGLE);
    }
    else if (xboxcontroller.getXboxDpad() == 0) {
      CalculateArmPID(SetPointAngles.INTAKE_HUMAN_ANGLE);
    }
    // need an else statement once triggers ready.
  }
}