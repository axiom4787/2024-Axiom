package frc.robot.commands.swervedrive.auto;

import frc.robot.Constants.MechState;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class MShootCommand extends InstantCommand{
    private final RobotContainer robotContainer;

    public MShootCommand(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    @Override
    public void initialize() {
        robotContainer.currentMechStateHandler(MechState.mShoot);
    }
}
