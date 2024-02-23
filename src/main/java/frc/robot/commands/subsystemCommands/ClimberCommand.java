// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystemCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberCommand extends Command {
  private final Climber climber;
  private final XboxController controller;
  private final Joystick joystick;
  private final static double CLIMBER_SPEED = 0.2;

  /** Creates a new ClimberCommand. */
  public ClimberCommand(Climber passedClimber, XboxController passedController, Joystick passedJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    climber = passedClimber;
    controller = passedController;
    joystick = passedJoystick;

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
    
    boolean controlToJoystick = joystick.getRawAxis(3) < 0; //This is the throttle thing on the front of the joystick
    double joystickY = joystick.getRawAxis(1); //Y axis of joystick
    boolean leftButton = joystick.getRawButton(5) || joystick.getRawButton(3); // Both buttons on the left side- either should mean that left is being pressed
    boolean rightButton = joystick.getRawButton(6) || joystick.getRawButton(4); // Both buttons on the right side- either should mean that right is being pressed

    if (controlToJoystick) { //If control is delegated to joystick
      if (leftButton && rightButton) { //If both buttons are pressed, stop
        climber.moveClimbers(0.0, 0.0);
      } else if (leftButton) {
        //If y axis is outside deadzone and negative, move left climber down, else move up
        if (joystickY < -0.1) { //Checking if outside deadzone and negative (controller forwards is negative)
          climber.moveClimbers(-CLIMBER_SPEED, 0.0);
        } else if (joystickY > 0.1) { //Checking if outside deadzone and positive (controller backwards is positive)
          climber.moveClimbers(CLIMBER_SPEED, 0.0);
        } else { //If inside deadzone, stop
          climber.moveClimbers(0.0, 0.0);
        }
      } else if (rightButton) {
        //If y axis is outside deadzone and negative, move right climber down, else move up
        if (joystickY < -0.1) { //Checking if outside deadzone and negative (controller forwards is negative)
          climber.moveClimbers(0.0, -CLIMBER_SPEED);
        } else if (joystickY > 0.1) { //Checking if outside deadzone and positive (controller backwards is positive)
          climber.moveClimbers(0.0, CLIMBER_SPEED);
        } else { //If inside deadzone, stop
          climber.moveClimbers(0.0, 0.0);
        }
      } else { //If no buttons are pressed, stop
        climber.moveClimbers(0.0, 0.0);
      }
    } else if (leftTrigger > 0.1 || rightTrigger > 0.1) { //In case control not delegated to joystick: If triggers are outside deadzone, move climber
      if (leftTrigger > rightTrigger) {
        climber.moveClimbers(CLIMBER_SPEED, CLIMBER_SPEED); //If left trigger is pressed more, move both climbers up- left is faster
      }
      else {
        climber.moveClimbers(-CLIMBER_SPEED, -CLIMBER_SPEED); //If right trigger is pressed more, move both climbers down- left is faster
      }
    } else {
      climber.moveClimbers(0.0, 0.0); //If triggers are inside deadzone, stop
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
