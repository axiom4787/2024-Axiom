// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystemCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberCommand extends Command {
  private final Climber climber;
  private final XboxController controller;

  /** Creates a new ClimberCommand. */
  public ClimberCommand(Climber passedClimber, XboxController passedController) {
    // Use addRequirements() here to declare subsystem dependencies.
    climber = passedClimber;
    controller = passedController;

    addRequirements(passedClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftTrigger = controller.getLeftTriggerAxis();
    double rightTrigger = controller.getRightTriggerAxis();
    if (leftTrigger > 0.1 || rightTrigger > 0.1) {
      if (leftTrigger > rightTrigger) {
        climber.moveClimbers(0.2, 0);
      }
      else {
        climber.moveClimbers(-0.2, 0);
      }
    } else {
      climber.moveClimbers(0.0, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
