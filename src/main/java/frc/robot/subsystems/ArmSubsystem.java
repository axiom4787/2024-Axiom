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

  private static double kP = 0.000005;
  private static double kI = 0.0;
  private static double kD = 0.0;

  XboxController driverXbox = new XboxController(0);
  private PIDController armPID = new PIDController(kP, kI, kD);
    
  public ArmSubsystem() {

    armPID.enableContinuousInput(-180, 180);

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

  public void simCalculateArmPID(double kSetpoint) {
    armLeftMotor.setVoltage(MathUtil.clamp(armPID.calculate(armLeftEncoder.getPosition(), kSetpoint) + ArmIntakeShooter.FEED_FOWARD, 0, 1));
    armRightMotor.setVoltage(MathUtil.clamp(armPID.calculate(armLeftEncoder.getPosition(), kSetpoint) + ArmIntakeShooter.FEED_FOWARD, 0, 1));
    // armPID.reset();
  }
  public void calculateArmPID(double kSetpoint) {
    armLeftMotor.set(MathUtil.clamp(armPID.calculate(armLeftEncoder.getPosition(), kSetpoint) + ArmIntakeShooter.FEED_FOWARD, 0, 1));
    armRightMotor.set(MathUtil.clamp(armPID.calculate(armLeftEncoder.getPosition(), kSetpoint) + ArmIntakeShooter.FEED_FOWARD, 0, 1));
    // armPID.reset();
  }

  // public void ArmMove(double Movement){
  //   armLeftMotor.set(Movement/2.5); //Change this for more motor power
  // }

  public void setArmPID() {
    System.out.println(driverXbox.getPOV());
    if (driverXbox.getPOV() == 180) {
      calculateArmPID(SetPointAngles.INTAKE_GROUND_ANGLE);
      // armLeftMotor.set(1);
    }
    else if (driverXbox.getPOV() == 270) {
      calculateArmPID(SetPointAngles.SHOOTER_AMP_ANGLE);
    }
    else if (driverXbox.getPOV() == 90) {
      calculateArmPID(SetPointAngles.SHOOTER_SPEAKER_ANGLE);
    }
    else if (driverXbox.getPOV() == 0) {
      calculateArmPID(SetPointAngles.INTAKE_HUMAN_ANGLE);
    }
    else {
      armLeftMotor.set(0);
      armRightMotor.set(0);
    }
  }
  public void simSetArmPID() {
    // System.out.println(driverXbox.getPOV());
    if (driverXbox.getPOV() == 180) {
      simCalculateArmPID(SetPointAngles.INTAKE_GROUND_ANGLE);
      // armLeftMotor.set(1);
    }
    else if (driverXbox.getPOV() == 270) {
      simCalculateArmPID(SetPointAngles.SHOOTER_AMP_ANGLE);
    }
    else if (driverXbox.getPOV() == 90) {
      simCalculateArmPID(SetPointAngles.SHOOTER_SPEAKER_ANGLE);
    }
    else if (driverXbox.getPOV() == 0) {
      simCalculateArmPID(SetPointAngles.INTAKE_HUMAN_ANGLE);
      // System.out.println(driverXbox.getPOV());
    }
    else {
      armLeftMotor.setVoltage(0);
      armRightMotor.setVoltage(0);
      // System.out.println(driverXbox.getPOV());
    }
  }

  @Override
  public void periodic() {
  // This method will be called once per scheduler run
    // if (driverXbox.getPOV() == 180) {
    //   calculateArmPID(SetPointAngles.INTAKE_GROUND_ANGLE);
    // }
    // else if (driverXbox.getPOV() == 270) {
    //   calculateArmPID(SetPointAngles.SHOOTER_AMP_ANGLE);
    // }
    // else if (driverXbox.getPOV() == 90) {
    //   calculateArmPID(SetPointAngles.SHOOTER_SPEAKER_ANGLE);
    // }
    // else if (driverXbox.getPOV() == 0) {
    //   calculateArmPID(SetPointAngles.INTAKE_HUMAN_ANGLE);
    // }
    // else {
    //   armLeftMotor.set(0);
    //   System.out.println(driverXbox.getPOV());
    // }
    // need an else statement once triggers ready.
    simSetArmPID();
  }
}
