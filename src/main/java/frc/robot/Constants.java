// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

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

    public static final PIDConstants TranslationPID     = new PIDConstants(6, 0, 0);
    public static final PIDConstants angleAutoPID       = new PIDConstants(2.5, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
    public static final PIDController trackingPID = new PIDController(0.02, 0.0,
            0.0);
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    // Controller Ports
    public static final int DRIVER_CONTROLLER_PORT = 0;

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.5;
    public static final double LEFT_Y_DEADBAND = 0.5;
    public static final double RIGHT_X_DEADBAND = 0.5;
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

  public static final class ClimberConstants
  {
      public static final int kLeftCanId = 16; // neo
      public static final int kRightCanId = 12; // neo
      public static final double kClimberSpeed = 0.75;
  }

  public static final class ShooterConstants
  {
      public static final int kFrontCanId = 13; // neo
      public static final int kBackCanId = 15; // neo
      public static final int kTopIndexerCanId = 11; // redline

      public static final double kShooterLaunchSpeed = 1;//1;
      public static final double kShooterIntakeSpeed = -0.5;//-1;
      public static final double kTopIndexerLaunchSpeed = 1;//0.5;
      public static final double kTopIndexerIntakeSpeed = -0.5;//-0.5;
      public static final double kTopIndexerDelay = 2;

  }

  public static final class RollerConstants
  {
      public static final int kRollerCanId = 14; // neo
      public static final double kRollerSpeed = 0.2;
  }

  public static final class GroundIntakeConstants
  {
      public static final int kGroundIntakeCanId = 9; // neo
      public static final int kBottomIndexerCanId = 10; // neo 550
      public static final double kIntakeSpeed = 0.3;
      public static final double kBottomIndexerSpeed = 0.8;
  }

  public enum MechState
  {
      mOff,
      mShoot,
      mIntake, 
      mChargeShoot,
  }

  public enum CurrentMechState {
    mOff,
    mRoller,
    mShooter,
    mGroundIntake,
    mBoth, // Represents the state where both roller and shooter are active
}

}
