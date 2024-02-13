// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkBase.IdleMode;

public class Climber extends SubsystemBase {
  private CANSparkMax leftClimber, rightClimber; //Motor controllers of CANSparkMax type, represent the climbers in code
  private RelativeEncoder leftEncoder, rightEncoder; //Encoders- used to measure the distance the climbers have moved. Will be used both for safety and control
  private double leftStartingPos, rightStartingPos; //Starting positions- used to adjust so that any starting position can work with code.
  /** Creates a new Climber. */
  public Climber() {
    leftClimber = new CANSparkMax(Constants.Climber.LEFT_CLIMBER_ID, CANSparkMax.MotorType.kBrushless);
    rightClimber = new CANSparkMax(Constants.Climber.RIGHT_CLIMBER_ID, CANSparkMax.MotorType.kBrushless);

    leftEncoder = leftClimber.getEncoder();
    rightEncoder = rightClimber.getEncoder();

    rightClimber.setInverted(Constants.Climber.RIGHT_INVERTED);
    leftClimber.setInverted(Constants.Climber.LEFT_INVERTED);

    leftClimber.setIdleMode(IdleMode.kBrake);
    rightClimber.setIdleMode(IdleMode.kBrake);

    leftClimber.setSmartCurrentLimit(40);
    rightClimber.setSmartCurrentLimit(40);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Put starting values on smartdashboard
    SmartDashboard.putNumber("Left Climber Starting Position", leftStartingPos);
    SmartDashboard.putNumber("Right Climber Starting Position", rightStartingPos);
    //Put encoder values on smartdashboard
    SmartDashboard.putNumber("Left Climber Encoder", leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Climber Encoder", rightEncoder.getPosition());
    //Put encoder-starting values on smartdashboard
    SmartDashboard.putNumber("Left Climber Adjusted Pos", leftEncoder.getPosition()-leftStartingPos);
    SmartDashboard.putNumber("Right Climber Adjusted Pos", rightEncoder.getPosition()-rightStartingPos);
  }

  public void setClimbers(double leftSpeed, double rightSpeed) {
    leftClimber.set(leftSpeed);
    rightClimber.set(rightSpeed);
  }
}
