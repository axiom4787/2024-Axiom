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

<<<<<<< HEAD
  XboxController xboxcontroller = new XboxController(0);
=======
  private XboxController driverXbox = new XboxController(0);


>>>>>>> 9a118b73fb34be9a32b3799da7dc69eec0517216
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
<<<<<<< HEAD
    armLeftMotor.setVoltage(MathUtil.clamp(ArmPID.calculate(armLeftEncoder.getPosition(), kSetpoint) + ArmIntakeShooter.FEED_FOWARD, 0, 12));
    ArmPID.reset();
=======
    armLeftMotor.set(MathUtil.clamp(ArmPID.calculate(armLeftEncoder.getPosition(), kSetpoint) + ArmIntakeShooter.FEED_FOWARD, -0.1, 0.1));
    System.out.println(ArmPID.calculate(armLeftEncoder.getPosition(), kSetpoint));
  }

  public void SimCalculateArmPID(double kSetpoint) {
    armLeftMotor.setVoltage(MathUtil.clamp(ArmPID.calculate(armLeftEncoder.getPosition(), kSetpoint) + ArmIntakeShooter.FEED_FOWARD, -0.1, 0.1));
    System.out.println(ArmPID.calculate(armLeftEncoder.getPosition(), kSetpoint));
>>>>>>> 9a118b73fb34be9a32b3799da7dc69eec0517216
  }

  // public void ArmMove(double Movement){
  //   armLeftMotor.set(Movement/2.5); //Change this for more motor power
  // }

  public void setArmPID() {
    System.out.println(driverXbox.getPOV());
    if (driverXbox.getPOV() == 180) {
      CalculateArmPID(SetPointAngles.INTAKE_GROUND_ANGLE);
      // armLeftMotor.set(1);
    }
    else if (driverXbox.getPOV() == 270) {
      CalculateArmPID(SetPointAngles.SHOOTER_AMP_ANGLE);
    }
    else if (driverXbox.getPOV() == 90) {
      CalculateArmPID(SetPointAngles.SHOOTER_SPEAKER_ANGLE);
    }
    else if (driverXbox.getPOV() == 0) {
      CalculateArmPID(SetPointAngles.INTAKE_HUMAN_ANGLE);
    }
    else {armLeftMotor.set(0);}
  }
  public void SimSetArmPID() {
    System.out.println(driverXbox.getPOV());
    if (driverXbox.getPOV() == 180) {
      SimCalculateArmPID(SetPointAngles.INTAKE_GROUND_ANGLE);
      // armLeftMotor.set(1);
    }
    else if (driverXbox.getPOV() == 270) {
      SimCalculateArmPID(SetPointAngles.SHOOTER_AMP_ANGLE);
    }
    else if (driverXbox.getPOV() == 90) {
      SimCalculateArmPID(SetPointAngles.SHOOTER_SPEAKER_ANGLE);
    }
    else if (driverXbox.getPOV() == 0) {
      SimCalculateArmPID(SetPointAngles.INTAKE_HUMAN_ANGLE);
    }
    else {armLeftMotor.setVoltage(0);}
  }

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
