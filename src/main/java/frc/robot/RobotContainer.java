// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.MechState;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.CurrentMechState;
import frc.robot.commands.swervedrive.auto.MChargeShootCommand;
import frc.robot.commands.swervedrive.auto.MIntakeCommand;
import frc.robot.commands.swervedrive.auto.MMGroundIntakeCommand;
import frc.robot.commands.swervedrive.auto.MMRollerCommand;
import frc.robot.commands.swervedrive.auto.MMShooterCommand;
import frc.robot.commands.swervedrive.auto.MOffCommand;
import frc.robot.commands.swervedrive.auto.MShootCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.MechanismSubsystem;
import frc.robot.subsystems.SimulatedLimelightData;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.sql.Date;
import java.time.Instant;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.opencv.core.Mat;

import com.ctre.phoenix.Util;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Pose2d simulatedAprilTag = new Pose2d(5.0, 5.0, new Rotation2d(200));

  // The robot's subsystems and commands are defined here...
  private final LimelightHelpers limelight = new LimelightHelpers();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                          "swerve/neo"));
  // private final ClimberSubsystem climber = new ClimberSubsystem();

  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // CommandJoystick driverController = new CommandJoystick(0);

  MechanismSubsystem kitBotMechanism = new MechanismSubsystem();

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(0);
  Joystick backupJoystick = new Joystick(1);

  private final SendableChooser<Command> autoChooser;
  private final Boolean lockToAprilTagBool = false;
  private CurrentMechState currentMechState = CurrentMechState.mShooter;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX(),
        () -> driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX());

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedAnglularVelocity);
    
    // drivebase.setDefaultCommand(closedFieldAbsoluteDrive);

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


  // public double calculateTrackingAngularVelocity(double rot) {
  // //   /*
  // //    * if (LL.getXAngle() != 0 && Math.abs(LL.getXAngle()) >= 1) {
  // //    * double speed = 0.03; // between 0 amd 1
  // //    * double direction = (-LL.getXAngle()) / Math.abs(LL.getXAngle());
  // //    * double scaleFactor = (Math.abs(LL.getXAngle())) * speed;
  // //    * SmartDashboard.putNumber("tracking velocity", direction * scaleFactor);
  // //    * if (scaleFactor > 2) {
  // //    * scaleFactor = 1.4;
  // //    * }
  // //    * return direction * scaleFactor;
  // //    * }
  // //    * 
  // //    * return 0;
  // //    */

  // //   // SimulatedLimelightData simulatedLimelightData = calculateSimulatedLimelightValues(); 
  // //   // double simulatedXAngle = simulatedLimelightData.xAngleToTag;

  //   if (rot != 0) {
  //       return rot;
  //   }

  //   if (limelight.getXAngle() != 0) {
  //     double pidOutput = Constants.Auton.trackingPID.calculate(limelight.getXAngle(), 0);
  //     return MathUtil.clamp(pidOutput, -1, 1);
  //   }

  //   return 0;
  // }

  // public double calculateTrackingXVelocity(double xVelocity) {
  //   // SimulatedLimelightData simulatedLimelightData = calculateSimulatedLimelightValues(); 
  //   // double simulatedXAngleOfRobot = simulatedLimelightData.robotAngleInTagSpace;

  //   if (xVelocity != 0) {
  //       return xVelocity;
  //   }

  //   // limelight.getCamPose2dInTargetSpace().getRotation().getDegrees()
  //   if (limelight.getCamPose2dInTargetSpace().getRotation().getDegrees() != 0) {
  //     double pidOutput = Constants.Auton.trackingPID.calculate(limelight.getCamPose2dInTargetSpace().getRotation().getDegrees(), 0);
  //     return MathUtil.clamp(pidOutput, -1, 1);
  //   }
  //   return 0;
  // }

  private Command commandConsumer(Supplier<Command> consumer) {
    return consumer.get();
  }

  private Command createCommandForMechanism(Consumer<MechState> setStateFunction, MechState state) {
    return new InstantCommand(() -> setStateFunction.accept(state), kitBotMechanism);
  }

  public void currentMechStateHandler(MechState mechState) {
    SmartDashboard.putString("currentMechStateHandler mechState", String.valueOf(mechState));
    // SmartDashboard.putString("Current Time", String.valueOf(System.currentTimeMillis()));
    // SmartDashboard.putString("currentMechStateHandler CMS", currentMechState.toString());
    if (currentMechState == CurrentMechState.mRoller) {
        createCommandForMechanism(kitBotMechanism::setRollerState, mechState).schedule();
    } else if (currentMechState == CurrentMechState.mShooter) {
        createCommandForMechanism(kitBotMechanism::setShooterState, mechState).schedule();
    } else if (currentMechState == CurrentMechState.mGroundIntake){
        createCommandForMechanism(kitBotMechanism::setGroundIntakeState, mechState).schedule();
    } else if (currentMechState == CurrentMechState.mBoth) {
        // Handle both roller and shooter states
        new InstantCommand(() -> kitBotMechanism.setBothStates(mechState, mechState)).schedule();
    } else {
        // If it's neither roller, shooter, nor both, or you need to reset both
        Command rollerOffCommand = createCommandForMechanism(kitBotMechanism::setRollerState, MechState.mOff);
        Command shooterOffCommand = createCommandForMechanism(kitBotMechanism::setShooterState, MechState.mOff);
        new SequentialCommandGroup(rollerOffCommand, shooterOffCommand).schedule();;
    }
  } 

  public void setCurrentMechState(CurrentMechState state) {
    currentMechState = state;
    SmartDashboard.putString("CMS", currentMechState.toString());  
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
    // Boolean isDriving = MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND) != 0
    //                     || MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND) != 0
    //                     || MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.LEFT_X_DEADBAND) != 0;
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new JoystickButton(driverXbox, Button.kY.value).onTrue((new RunCommand(drivebase::zeroModules, drivebase)));
    // new JoystickButton(driverXbox, Button.kY.value).onTrue((new RunCommand(()-> testSubsystem.move(0.5), testSubsystem)));
    // new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
    // new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    // new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
    // Add a button to run pathfinding commands to SmartDashboard
    // Button binding based on the numbered box from the left.

    // Command mIntakeCommand = new InstantCommand(() -> currentMechStateHandler(MechState.mIntake));
    // Command mShootCommand = new InstantCommand(() -> currentMechStateHandler(MechState.mShoot));
    // Command mChargeShootCommand = new InstantCommand(() -> currentMechStateHandler(MechState.mChargeShoot));
    // Command mOffCommand = new InstantCommand(() -> currentMechStateHandler(MechState.mOff));
    // Command mmRollerCommand = new InstantCommand(() -> setCurrentMechState(CurrentMechState.mRoller), kitBotMechanism);
    // Command mmShooterCommand = new InstantCommand(() -> setCurrentMechState(CurrentMechState.mShooter), kitBotMechanism);
    // Command mmGroundIntakeCommand = new InstantCommand(() -> setCurrentMechState(CurrentMechState.mGroundIntake), kitBotMechanism);

    MIntakeCommand mIntakeCommand = new MIntakeCommand(this);
    MShootCommand mShootCommand = new MShootCommand(this);
    MChargeShootCommand mChargeShootCommand = new MChargeShootCommand(this);
    MOffCommand mOffCommand = new MOffCommand(this);
    MMRollerCommand mmRollerCommand = new MMRollerCommand(this);
    MMShooterCommand mmShooterCommand = new MMShooterCommand(this);
    MMGroundIntakeCommand mmGroundIntakeCommand = new MMGroundIntakeCommand(this);

    // List<Pair<String, Command>> commandList = new ArrayList<>();
    // commandList.add(new Pair<>("mIntake", mIntakeCommand));
    // commandList.add(new Pair<>("mShoot", mShootCommand));
    // commandList.add(new Pair<>("mChargeShoot", mChargeShootCommand));
    // commandList.add(new Pair<>("mOff", mOffCommand));
    // commandList.add(new Pair<>("mmRoller", mmRollerCommand));
    // commandList.add(new Pair<>("mmShooter", mmShooterCommand));
    // commandList.add(new Pair<>("mmGroundIntake", mmGroundIntakeCommand));

    // NamedCommands.registerCommands(commandList);   
    //register individual commands
    NamedCommands.registerCommand("mIntake", mIntakeCommand);
    NamedCommands.registerCommand("mShoot", mShootCommand);
    NamedCommands.registerCommand("mChargeShoot", mChargeShootCommand);
    NamedCommands.registerCommand("mOff", mOffCommand);
    NamedCommands.registerCommand("mmRoller", mmRollerCommand);
    NamedCommands.registerCommand("mmShooter", mmShooterCommand);
    NamedCommands.registerCommand("mmGroundIntake", mmGroundIntakeCommand);
    // NamedCommands.registerCommand("shooting", new SequentialCommandGroup(
    //   mmShooterCommand,
    //   mChargeShootCommand,
    //   new WaitCommand(Constants.ShooterConstants.kTopIndexerDelay),
    //   mShootCommand,
    //   new WaitCommand(0.4),
    //   mOffCommand
    // ));


    new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(() -> drivebase.zeroGyro()));
    new JoystickButton(driverXbox, 5).onTrue(mmRollerCommand);
    new JoystickButton(driverXbox, 6).onTrue(mmShooterCommand);
    new JoystickButton(driverXbox, 2).onTrue(mmGroundIntakeCommand);
    // use joysticktriggers to set current mech state to roller (left trigger) or shooter (right trigger) and when let go, set to off with driverXbox.getLeftTriggerReleased() or driverXbox.getRightTriggerReleased()
    // Left Trigger for Roller
    // Left Trigger for Intake
    new Trigger(() -> driverXbox.getLeftTriggerAxis() > 0.5)
    .onTrue(mIntakeCommand);

    // // Right Trigger for Shoot
    new Trigger(() -> driverXbox.getRightTriggerAxis() > 0.5)
    .onTrue(
      mChargeShootCommand
      .andThen((new WaitCommand(Constants.ShooterConstants.kTopIndexerDelay)))
      .andThen(mShootCommand)
      .until(() -> driverXbox.getRightTriggerAxis() < 0.5)
    );

    // // when both are not pressed, set to off
    new Trigger(() -> driverXbox.getLeftTriggerAxis() < 0.5 && driverXbox.getRightTriggerAxis() < 0.5)
    .onTrue(mOffCommand);

    BooleanSupplier isMoving = () -> Math.abs(driverXbox.getLeftY()) > OperatorConstants.LEFT_Y_DEADBAND 
                                  || Math.abs(driverXbox.getLeftX()) > OperatorConstants.LEFT_X_DEADBAND 
                                  || Math.abs(driverXbox.getRightX()) > OperatorConstants.RIGHT_X_DEADBAND;
    
    // Supplier<Command> ampCommand = () -> new SequentialCommandGroup(
    //   new SequentialCommandGroup(
    //       AutoBuilder.pathfindToPose(
    //           new Pose2d(1.86, 7.41, Rotation2d.fromDegrees(-90)),
    //           new PathConstraints(1, 1.0, Units.degreesToRadians(540), Units.degreesToRadians(720)),
    //           0,
    //           0
    //       ),
    //       new SequentialCommandGroup(
    //           new InstantCommand(() -> currentMechState = CurrentMechState.mRoller),
    //           new InstantCommand(() -> currentMechStateHandler(MechState.mShoot))
    //           .andThen(new WaitCommand(Constants.ShooterConstants.kTopIndexerDelay))
    //       )
    //   ),
    //   new InstantCommand(() -> currentMechStateHandler(MechState.mOff))
    // ).until(isMoving);                       

    // Supplier<Command> speakerCommand = () -> new SequentialCommandGroup(
    //   new SequentialCommandGroup(
    //       AutoBuilder.pathfindToPose(
    //           new Pose2d(1.48, 5.49, Rotation2d.fromDegrees(180)), 
    //           new PathConstraints(1, 1.0, Units.degreesToRadians(540), Units.degreesToRadians(720)), 
    //           0, 0
    //       ),
    //       new SequentialCommandGroup(
    //           new InstantCommand(() -> currentMechState = CurrentMechState.mShooter),
    //           new WaitCommand(0.8),
    //           new InstantCommand(() -> currentMechStateHandler(MechState.mShoot)).withTimeout(0.8)
    //       )
    //   ),
    //   new InstantCommand(() -> currentMechStateHandler(MechState.mOff))
    // ).until(isMoving);

    // Supplier<Command> humanIntakeCommand = () -> new SequentialCommandGroup(
    //   AutoBuilder.pathfindToPose(
    //     new Pose2d(14.78, 0.65, Rotation2d.fromDegrees(-53.96)), 
    //     new PathConstraints(
    //       1, 1.0, 
    //       Units.degreesToRadians(540), Units.degreesToRadians(720)
    //     ), 
    //     0, 
    //     0
    //   ),
    //   new InstantCommand(() -> currentMechStateHandler(MechState.mOff))
    // ).until(isMoving);

    // Command speakerCycle = new SequentialCommandGroup(
    //   commandConsumer(humanIntakeCommand),
    //   commandConsumer(speakerCommand)
    // );

    // Command ampCycle = new SequentialCommandGroup(
    //   commandConsumer(humanIntakeCommand),
    //   commandConsumer(ampCommand)
    // );

    // SmartDashboard.putData("Pathfind to Amp", commandConsumer(ampCommand));

    // SmartDashboard.putData("Pathfind to Speaker", commandConsumer(speakerCommand));

    // SmartDashboard.putData("Pathfind to Human Intake", commandConsumer(humanIntakeCommand));

    // SmartDashboard.putData("Speaker Cycle", speakerCycle);

    // SmartDashboard.putData("Amp Cycle", ampCycle);

    // SmartDashboard.putData("3 Piece Auto", AutoBuilder.buildAuto(
    //   "3PIece"
    // ));

    // SmartDashboard.putData("move forward", drivebase.getAutonomousCommand("move forward"));


    // SmartDashboard.putData("move forward", AutoBuilder.followPath(
    //   PathPlannerPath.fromPathFile("test")
    // ));

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
          4.12, 4.0, 
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
    //return autoChooser.getSelected();
    return drivebase.getAutonomousCommand("1Piece");
    //return Commands.none();
  }

  // public Command getTeleopCommand()
  // {
  //   // An example command will be run in autonomous
  //   // return drivebase.getAutonomousCommand("New Path", true);
  //   //return autoChooser.getSelected();
  //   return climberCommand;
  // }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    //drivebase.setMotorBrake(brake);
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
  

  // public RunCommand getArmCommand() {
  //   return new RunCommand(() -> armSubsystem.setArmPID(), armSubsystem);
  // }

  public void updateDashboard() {
    SmartDashboard.putString("Current Mech State", currentMechState.toString());
  }
}