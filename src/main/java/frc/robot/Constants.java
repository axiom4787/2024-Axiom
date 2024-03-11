// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final int XCONTROLLER_PORT = 1;

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
    public static final PIDController trackingPID = new PIDController(0.02, 0.0,
            0.0);
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class ArmIntakeShooter 
  {
    public static final int TOP_MOTORID = 12; // neo
    public static final int BOTTOM_MOTORID = 11; //neo 550
    public static final int LEFTARM_MOTORID = 10; //neo 
    public static final int RIGHTARM_MOTORID = 9; //neo
    
    public static final int TOP_MOTOR_CURRENT_LIMIT = 40; // neo amp
    public static final int BOTTOM_MOTOR_CURRENT_LIMIT = 20; // neo 550 amp
    public static final int LEFTARM_MOTOR_CURRENT_LIMIT = 40; //neo amp
    public static final int RIGHTARM_MOTOR_CURRENT_LIMIT = 40; // neo amp

    public static final double SHOOT_DELAY = 0.5;
    
    // public static final IdleMode TOP_MOTOR_IDLE_MODE = IdleMode.kCoast;
    // public static final IdleMode BOTTOM_MOTOR_IDLE_MODE = IdleMode.kBrake;
    // public static final IdleMode LEFTARM_MOTOR_IDLE_MODE = IdleMode.kBrake;
    // public static final IdleMode RIGHTMOTOR_MOTOR_IDLE_MODE = IdleMode.kBrake;

    public static final double FEED_FOWARD = 0.0;
  }

  public static final class MotorTestConstants
  {
    public static final int MOTOR_ID = 13; // neo
  }

  public static final class SetPointAngles
  {
    public static final int INTAKE_GROUND_ANGLE = 0;
    public static final int INTAKE_HUMAN_ANGLE = -70; //Really goofy method of intaking, just get it in and push back and forth until it falls in right- should mainly ground intake
    public static final int SHOOTER_AMP_ANGLE = -165; //REMEMBER THE BUMPERS
    public static final int SHOOTER_SPEAKER_ANGLE = -70;
  }

  public static final class Climber
  {
    public static final int LEFT_CLIMBER_ID = 14;
    public static final int RIGHT_CLIMBER_ID = 13;
    public static final boolean LEFT_INVERTED = false;
    public static final boolean RIGHT_INVERTED = true;
  }

  public static class OperatorConstants
  {
    // Controller Ports
    public static final int DRIVER_CONTROLLER_PORT = 0;

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.2;
    public static final double LEFT_Y_DEADBAND = 0.2;
    public static final double RIGHT_X_DEADBAND = 0.2;
    public static final double TURN_CONSTANT = 0.2;
  }

  public static final class LimelightConstants {
    public static final double kLLHeight = Units.inchesToMeters(24.5);
    public static final double kLLPitch = Units.degreesToRadians(0);
    public static final double kMinObjectAvoidanceDistance = Units.inchesToMeters(12);
    public static final double kObjectHeight = Units.inchesToMeters(12);
    public static final double kObjectPitch = Units.degreesToRadians(0);

    public static final int[] kBlueAprilTags = new int[]{1,2,3,4};
    public static final int[] kRedAprilTags = new int[]{5,6,7,8};
  }

  public static final class BlinkinConstants {
    public static final int kPWMPort = 0;
    public static final double kRainbow = -0.99;
    public static final double kSolidOrange = 0.65;
    public static final double kAllianceColor = -0.01;
  }
}
