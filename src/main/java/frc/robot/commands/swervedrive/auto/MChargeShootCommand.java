package frc.robot.commands.swervedrive.auto;

import frc.robot.Constants.MechState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class MChargeShootCommand extends InstantCommand{
    private final RobotContainer robotContainer;

    public MChargeShootCommand(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    @Override
    public void initialize() {
        robotContainer.currentMechStateHandler(MechState.mChargeShoot);
    }
}
