// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SetPointAngles;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.MotorTest;
import frc.robot.subsystems.ShooterIntake;
import frc.robot.subsystems.SimulatedLimelightData;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.ShooterIntake;
import java.io.File;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.subsystems.ArmSubsystem; 

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Pose2d simulatedAprilTag = new Pose2d(5.0, 5.0, new Rotation2d(200));

  // The robot's subsystems and commands are defined here...
  private final LimeLight limelight = new LimeLight();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"), limelight);
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // CommandJoystick driverController = new CommandJoystick(0);

  ShooterIntake shooterIntake = new ShooterIntake();
  ArmSubsystem armSubsystem = new ArmSubsystem();
  MotorTest testSubsystem = new MotorTest();

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(0);

  private final SendableChooser<Command> autoChooser;
  private final Boolean lockToAprilTagBool = false;

  private RunCommand offStateDirection = new RunCommand(() -> shooterIntake.setStateDirection("off"), shooterIntake);
  private RunCommand offStateLocation = new RunCommand(() -> shooterIntake.setStateLocation("off"), shooterIntake);
  private ParallelCommandGroup offShooterIntake = new ParallelCommandGroup(offStateDirection, offStateLocation);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
                                                          // Applies deadbands and inverts controls because joysticks
                                                          // are back-right positive while robot
                                                          // controls are front-left positive
                                                          () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                       OperatorConstants.LEFT_Y_DEADBAND),
                                                          () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                       OperatorConstants.LEFT_X_DEADBAND),
                                                          () -> -driverXbox.getRightX(),
                                                          () -> -driverXbox.getRightY());

    AbsoluteFieldDrive closedFieldAbsoluteDrive = new AbsoluteFieldDrive(drivebase,
                                                                         () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                    OperatorConstants.LEFT_Y_DEADBAND)/2,
                                                                         () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                      OperatorConstants.LEFT_X_DEADBAND)/2,
                                                                         () -> driverXbox.getRightX()/2);

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                      () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND)/3,
                                                                      () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                  OperatorConstants.LEFT_X_DEADBAND)/3,
                                                                      () -> MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                  OperatorConstants.RIGHT_X_DEADBAND)/3, 
                                                                      driverXbox::getYButtonPressed, 
                                                                      driverXbox::getAButtonPressed, 
                                                                      driverXbox::getXButtonPressed, 
                                                                      driverXbox::getBButtonPressed);

    TeleopDrive simClosedFieldRel = new TeleopDrive(drivebase,
                                                    () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                 OperatorConstants.LEFT_Y_DEADBAND),
                                                    () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                 OperatorConstants.LEFT_X_DEADBAND),
                                                    () -> -driverXbox.getRawAxis(4), () -> true);
    // TeleopDrive closedFieldRel = new TeleopDrive(
    //     drivebase,
    //     () -> -MathUtil.applyDeadband(driverController.getY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> -MathUtil.applyDeadband(driverController.getX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND), () -> true);

    // drivebase.setDefaultCommand(RobotBase.isSimulation() ? closedAbsoluteDrive : closedFieldAbsoluteDrive);
    drivebase.setDefaultCommand(closedFieldAbsoluteDrive);

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  
  public SimulatedLimelightData calculateSimulatedLimelightValues() {
    // Get the robot's current pose
    Pose2d currentPose = drivebase.getPose();

    // Calculate the angle to the simulatedAprilTag
    double angleToTag = Math.atan2(simulatedAprilTag.getY() - currentPose.getY(),
                                   simulatedAprilTag.getX() - currentPose.getX());
    double robotHeading = currentPose.getRotation().getRadians();
    angleToTag -= robotHeading; // Adjust for robot's current heading

    // Calculate the distance to the simulatedAprilTag
    double distanceToTag = Math.hypot(simulatedAprilTag.getX() - currentPose.getX(),
                                      simulatedAprilTag.getY() - currentPose.getY());

    // Normalize angle to [-180, 180] range
    double xAngleToTag = Math.toDegrees(angleToTag);
    if (xAngleToTag > 180) xAngleToTag -= 360;
    if (xAngleToTag < -180) xAngleToTag += 360;

    // Check if the tag is within the Limelight's FOV (80 degrees)
    boolean isTargetVisible = Math.abs(xAngleToTag) <= 40; // 80-degree FOV divided by 2

    // Calculate the robot's offset from where the AprilTag is facing
    // Assuming AprilTag is facing along positive Y-axis
    double tagFacingAngle = Math.toDegrees(Math.atan2(1.0, 0.0)); // 90 degrees or pi/2 radians
    double robotOffsetFromTagFacing = robotHeading - Math.toRadians(tagFacingAngle);
    // Normalize to [-180, 180] range
    robotOffsetFromTagFacing = Math.toDegrees(robotOffsetFromTagFacing);
    if (robotOffsetFromTagFacing > 180) robotOffsetFromTagFacing -= 360;
    if (robotOffsetFromTagFacing < -180) robotOffsetFromTagFacing += 360;

    return new SimulatedLimelightData(xAngleToTag, robotOffsetFromTagFacing, distanceToTag, isTargetVisible);
  }


  public double calculateTrackingAngularVelocity(double rot) {
    /*
     * if (LL.getXAngle() != 0 && Math.abs(LL.getXAngle()) >= 1) {
     * double speed = 0.03; // between 0 amd 1
     * double direction = (-LL.getXAngle()) / Math.abs(LL.getXAngle());
     * double scaleFactor = (Math.abs(LL.getXAngle())) * speed;
     * SmartDashboard.putNumber("tracking velocity", direction * scaleFactor);
     * if (scaleFactor > 2) {
     * scaleFactor = 1.4;
     * }
     * return direction * scaleFactor;
     * }
     * 
     * return 0;
     */

    // SimulatedLimelightData simulatedLimelightData = calculateSimulatedLimelightValues(); 
    // double simulatedXAngle = simulatedLimelightData.xAngleToTag;

    if (rot != 0) {
        return rot;
    }

    if (limelight.getXAngle() != 0) {
      double pidOutput = Constants.Auton.trackingPID.calculate(limelight.getXAngle(), 0);
      return MathUtil.clamp(pidOutput, -1, 1);
    }

    return 0;
  }

  public double calculateTrackingXVelocity(double xVelocity) {
    // SimulatedLimelightData simulatedLimelightData = calculateSimulatedLimelightValues(); 
    // double simulatedXAngleOfRobot = simulatedLimelightData.robotAngleInTagSpace;

    if (xVelocity != 0) {
        return xVelocity;
    }

    // limelight.getCamPose2dInTargetSpace().getRotation().getDegrees()
    if (limelight.getCamPose2dInTargetSpace().getRotation().getDegrees() != 0) {
      double pidOutput = Constants.Auton.trackingPID.calculate(limelight.getCamPose2dInTargetSpace().getRotation().getDegrees(), 0);
      return MathUtil.clamp(pidOutput, -1, 1);
    }
    return 0;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new JoystickButton(driverXbox, Button.kY.value).onTrue((new RunCommand(drivebase::zeroModules, drivebase)));
    // new JoystickButton(driverXbox, Button.kY.value).onTrue((new RunCommand(()-> testSubsystem.move(0.5), testSubsystem)));
    // new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
    // new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    // new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
    // Add a button to run pathfinding commands to SmartDashboard
    // Button binding based on the numbered box from the left.
    new JoystickButton(driverXbox, 5).onTrue(new RunCommand(() -> shooterIntake.setStateDirection("intake"), shooterIntake));
    new JoystickButton(driverXbox, 6).onTrue(new RunCommand(() -> shooterIntake.setStateDirection("shoot"), shooterIntake));
    new JoystickButton(driverXbox, 1).onTrue(new RunCommand(() -> shooterIntake.setStateDirection("off"), shooterIntake).alongWith(new RunCommand(() -> shooterIntake.setStateLocation("off"), shooterIntake)));
    new JoystickButton(driverXbox, 7).onTrue(new RunCommand(() -> shooterIntake.setStateLocation("amp"), shooterIntake));
    new JoystickButton(driverXbox, 8).onTrue(new RunCommand(() -> shooterIntake.setStateLocation("speaker"), shooterIntake));
    new JoystickButton(driverXbox, driverXbox.getPOV()).onTrue(new RunCommand(() -> armSubsystem.simSetArmPID(), armSubsystem));
    SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
      new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0, 
      2.0
    ));
    SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
      new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)), 
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0, 
      0
    ));

    SmartDashboard.putData("Pathfind to AmpCycle", AutoBuilder.pathfindThenFollowPath(
      PathPlannerPath.fromPathFile("PathFindCycleToAmp"), 
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0
    ));

    // TeleopDrive lockToAprilTag = new TeleopDrive(
    //     drivebase,
    //     () -> -MathUtil.applyDeadband(driverController.getY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> -calculateTrackingXVelocity(MathUtil.applyDeadband(driverController.getX(), OperatorConstants.LEFT_X_DEADBAND)),
    //     () -> -calculateTrackingAngularVelocity(-driverController.getRawAxis(4)), () -> false);
    // SmartDashboard.putData("lock to tag", lockToAprilTag);

    // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // This example will simply move the robot 2m forward of its current position
    SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
      Pose2d currentPose = drivebase.getPose();
      
      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());

      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
        bezierPoints, 
        new PathConstraints(
          4.0, 4.0, 
          Units.degreesToRadians(360), Units.degreesToRadians(540)
        ),  
        new GoalEndState(0.0, currentPose.getRotation())
      );

      AutoBuilder.followPathWithEvents(path).schedule();
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    // return drivebase.getAutonomousCommand("New Path", true);
    return autoChooser.getSelected();
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }


  // public void setArmPID() {
  //   System.out.println(driverXbox.getPOV());
  //   if (driverXbox.getPOV() == 180) {
  //     armSubsystem.CalculateArmPID(SetPointAngles.INTAKE_GROUND_ANGLE);
  //   }
  //     armSubsystem.CalculateArmPID(SetPointAngles.SHOOTER_AMP_ANGLE);
  //   }
  //   else if (driverXbox.getPOV() == 90) {
  //     armSubsystem.CalculateArmPID(SetPointAngles.SHOOTER_SPEAKER_ANGLE);
  //   }
  //   else if (driverXbox.getPOV() == 0) {
  //     armSubsystem.CalculateArmPID(SetPointAngles.INTAKE_HUMAN_ANGLE);
  //   }
  //   else {armSubsystem.armLeftMotor.set(0);}
  // }
  

  public RunCommand getArmCommand() {
    return new RunCommand(() -> armSubsystem.setArmPID(), armSubsystem);
  }
}