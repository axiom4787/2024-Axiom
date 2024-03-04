// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
//Import talonfx
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkBase.IdleMode;

public class Climber extends SubsystemBase {
  private TalonFX leftClimber, rightClimber; //Motor controllers of CANSparkMax type, represent the climbers in code
  private double leftEncoderVal, rightEncoderVal; //Encoders- used to measure the distance the climbers have moved. Will be used both for safety and control
  private double leftStartingPos, rightStartingPos; //Starting positions- used to adjust so that any starting position can work with code.
  /** Creates a new Climber. */
  public Climber() {
    leftClimber = new TalonFX(Constants.Climber.LEFT_CLIMBER_ID);
    rightClimber = new TalonFX(Constants.Climber.RIGHT_CLIMBER_ID);

    rightClimber.setInverted(Constants.Climber.RIGHT_INVERTED);
    leftClimber.setInverted(Constants.Climber.LEFT_INVERTED);

    leftClimber.setNeutralMode(NeutralModeValue.Brake);
    rightClimber.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    leftEncoderVal = leftClimber.getPosition().getValueAsDouble();
    rightEncoderVal = rightClimber.getPosition().getValueAsDouble();

    //Put encoder values on smartdashboard
    SmartDashboard.putNumber("Left Climber Encoder", leftEncoderVal);
    SmartDashboard.putNumber("Right Climber Encoder", rightEncoderVal);
  }

  /**
   * This method moves the climbers at specified speed
   * @param leftSpeed
   * @param rightSpeed
   */
  public void moveClimbers(double leftSpeed, double rightSpeed) {
    leftClimber.set(leftSpeed);
    rightClimber.set(rightSpeed);
  }

  /**
   * @deprecated    
   * DEPRECATED- RIGHT NOW ONLY RETURNS 0
   * <p>
   * <p>
   * This method gets the ADJUSTED position of the left climber
   * <p>
   * Adjusted based on starting position
   * @return the adjusted position of the left climber
   */
  public double getAdjustedLeftPos() {
    return 0;
  }

  /**
   * @deprecated
   * DEPRECATED- RIGHT NOW ONLY RETURNS 0
   * <p>
   * <p>
   * This method gets the ADJUSTED position of the right climber
   * <p>
   * Adjusted based on starting position
   * @return the adjusted position of the right climber
   */
  public double getAdjustedRightPos() {
    return 0;
  }

  /**
   * SHOULD NOT USE (unless you know what you're doing)
   * <p>
   * This method gets the UNADJUSTED position of the left climber
   * <p>
   * Is NOT adjusted based on starting position
   * @return the unadjusted position of the left climber
   */
  public double getUnadjustedLeftPos() { 
    return leftEncoderVal;
  }

  /**
   * SHOULD NOT USE (unless you know what you're doing)
   * <p>
   * This method gets the UNADJUSTED position of the right climber
   * <p>
   * Is NOT adjusted based on starting position
   * @return the unadjusted position of the right climber
   */
  public double getUnadjustedRightPos() {
    return rightEncoderVal;
  }  
}