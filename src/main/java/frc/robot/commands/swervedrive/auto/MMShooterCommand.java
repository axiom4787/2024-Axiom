package frc.robot.commands.swervedrive.auto;

import frc.robot.Constants.CurrentMechState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class MMShooterCommand extends InstantCommand{
    private final RobotContainer robotContainer;

    public MMShooterCommand(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    @Override
    public void initialize() {
        robotContainer.setCurrentMechState(CurrentMechState.mShooter);
    }
}
