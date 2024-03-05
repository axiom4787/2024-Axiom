package frc.robot.subsystems;

import java.util.concurrent.TransferQueue;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmIntakeShooter;
import frc.robot.Constants.SetPointAngles;
import edu.wpi.first.wpilibj.XboxController;

public class ArmSubsystem extends SubsystemBase {
  public final CANSparkMax armLeftMotor; //neo
  private final CANSparkMax armRightMotor; //neo

  private final RelativeEncoder armLeftEncoder; //encoder for left arm
  private final RelativeEncoder armRightEncoder; //encoder for right arm
  
  private static double kP = 0.1;
  private static double kI = 0.0;
  private static double kD = 0.0;

  private double ticks = 0.0;

  XboxController driverXbox = new XboxController(0);
  private PIDController armPID = new PIDController(kP, kI, kD);
  

  public ArmSubsystem() {

    armPID.enableContinuousInput(-180, 180);

    armLeftMotor = new CANSparkMax(ArmIntakeShooter.LEFTARM_MOTORID, CANSparkMax.MotorType.kBrushless);
    armRightMotor = new CANSparkMax(ArmIntakeShooter.RIGHTARM_MOTORID, CANSparkMax.MotorType.kBrushless);

    armLeftMotor.restoreFactoryDefaults();
    armRightMotor.restoreFactoryDefaults();

    armLeftMotor.setInverted(false);
    armRightMotor.setInverted(false);

    // armRightMotor.follow(armLeftMotor);

    armLeftMotor.setSmartCurrentLimit(ArmIntakeShooter.LEFTARM_MOTOR_CURRENT_LIMIT);
    armRightMotor.setSmartCurrentLimit(ArmIntakeShooter.RIGHTARM_MOTOR_CURRENT_LIMIT);

    armLeftMotor.setIdleMode(IdleMode.kBrake);
    armRightMotor.setIdleMode(IdleMode.kBrake);

    armLeftEncoder = armLeftMotor.getEncoder();
    armRightEncoder = armRightMotor.getEncoder();

    // REVPhysicsSim.getInstance().addSparkMax(armLeftMotor, DCMotor.getNEO(1));
    // REVPhysicsSim.getInstance().addSparkMax(armRightMotor, DCMotor.getNEO(1));
  }

  public void simCalculateArmPID(double kSetpoint) {
    ticks = 2.844444444444 *  kSetpoint;
    if (armLeftEncoder.getPosition() > ticks || armLeftEncoder.getPosition() < ticks) {
      armLeftMotor.setVoltage(MathUtil.clamp(armPID.calculate(armLeftEncoder.getPosition(), kSetpoint) + ArmIntakeShooter.FEED_FOWARD, -1, 1));
      armRightMotor.setVoltage(MathUtil.clamp(armPID.calculate(armLeftEncoder.getPosition(), kSetpoint) + ArmIntakeShooter.FEED_FOWARD, -1, 1));
    }
    else if (armLeftEncoder.getPosition() == ticks) {
      armLeftMotor.setVoltage(0);
      armRightMotor.setVoltage(0);
    }
  }
    // armPID.reset();

  private void calculateArmPID(double kSetpoint) {
    double convertedSetPoint = (42.0/360.0) *  kSetpoint;
    System.out.println("convertedSetPoint: " + convertedSetPoint);
    double pidOutput = armPID.calculate(armLeftEncoder.getPosition(), convertedSetPoint) + ArmIntakeShooter.FEED_FOWARD;
    double clampedPidOutput = MathUtil.clamp(pidOutput, -0.2, 0.2);
    System.out.println("pid output:" + pidOutput);
    armLeftMotor.set(clampedPidOutput);
    armRightMotor.set(-clampedPidOutput);
    
    System.out.println("armLeftEncoder ticks: " + armLeftEncoder.getPosition());
    // System.out.println("ticks: " + ticks);
    // armPID.reset();
  }

  // public void ArmMove(double Movement){
  //   armLeftMotor.set(Movement/2.5); //Change this for more motor power
  // }

  public void setArmPID() {
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

  public void moveArm(double speed) {
    armLeftMotor.set(speed);
    armRightMotor.set(-speed);
  }

  @Override
  public void periodic() {
    // System.out.println("encoder pos: " + armLeftEncoder.getPosition());
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
    setArmPID();
  }
}
