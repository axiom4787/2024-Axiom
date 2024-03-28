package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.MechState;

public class MIntakeCommand extends InstantCommand {
    private final RobotContainer robotContainer;

    public MIntakeCommand(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    @Override
    public void initialize() {
        robotContainer.currentMechStateHandler(MechState.mIntake);
    }
}