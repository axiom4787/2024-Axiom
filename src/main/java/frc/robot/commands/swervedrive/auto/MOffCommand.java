package frc.robot.commands.swervedrive.auto;

import frc.robot.Constants.MechState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class MOffCommand extends InstantCommand{
    private final RobotContainer robotContainer;

    public MOffCommand(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    @Override
    public void initialize() {
        robotContainer.currentMechStateHandler(MechState.mOff);
    }
}
