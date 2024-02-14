package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX; 
import com.ctre.phoenix6.hardware.WPI_VictorSPX; 

public class Climbing extends SubsystemBase{

    private CANSparkMax leftClimber, rightClimber;
    // private DifferentialDrive differentialDrive;
    private final RelativeEncoder leftClimberEncoder, rightClimberEncoder;

    // falcon
    public Climbing() {
        leftClimber = new CANSparkMax(Constants.LEFT_CLIMBER_ID);
        rightClimber = new CANSparkMax(Constants.RIGHT_CLIMBER_ID);

        leftClimber.setInverted(Constants.LEFT_CLIMBER_INVERTED);
        rightClimber.setInverted(Constants.RIGHT_CLIMBER_INVERTED);

        // leftClimber.follow(rightClimber);

        leftClimber.getSmartCurrentLimit(Constants.LEFT_CLIMBER_CURRENT_LIMIT);
        rightClimber.getSmartCurrentLimit(Constants.RIGHT_CLIMBER_CURRENT_LIMIT);

        leftClimber.setIdleMode(IdleMode.kBrake);
        rightClimber.setIdleMode(IdleMode.kBrake);

        leftClimberEncoder = leftClimber.getEncoder();
        rightClimberEncoder = rightClimber.getEncoder();
    }

    public void climb(double speed) {
        rightClimber.set(speed);
        leftClimber.set(speed);
    }
}