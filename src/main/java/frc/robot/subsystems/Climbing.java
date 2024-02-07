package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX; 
import com.ctre.phoenix6.hardware.WPI_VictorSPX; 

public class Climbing extends SubsystemBase{

    private WPI_VictorSPX leftClimberOne, leftClimberTwo, rightClimberOne, rightClimberTwo;
    private DifferentialDrive differentialDrive;

    // falcon
    public Climbing() {
        leftClimberOne = new WPI_VictorSPX(Constants.LEFT_CLIMBER_ONE_ID);
        leftClimberTwo = new WPI_VictorSPX(Constants.LEFT_CLIMBER_TWO_ID);
        rightClimberOne = new WPI_VictorSPX(Constants.RIGHT_CLIMBER_ONE_ID);
        rightClimberTwo = new WPI_VictorSPX(Constants.RIGHT_CLIMBER_TWO_ID);

        leftClimberOne.setInverted(Constants.LEFT_CLIMBER_INVERTED);
        rightClimberOne.setInverted(Constants.RIGHT_CLIMBER_INVERTED);

        // leftClimberOne.follow(rightClimberOne);
        // leftClimberTwo.follow(rightClimberTwo);

        // differentialDrive = new DifferentialDrive(leftClimberOne, leftClimberTwo);
    }

    public void climb(double up, double down) {
        leftClimberOne.set(up, down);
        leftClimberTwo.set(up, down);
        rightClimberOne.set(up, down);
        rightClimberTwo.set(up, down);
        // differentialDrive.set(up, down);
    }
}