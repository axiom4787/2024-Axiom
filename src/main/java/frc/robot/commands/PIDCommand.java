package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;


public class PIDCommand extends Command{
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;
    private final double kSetpoint;
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    public PIDController pid;

    public PIDCommand(double kP, double kI, double kD, double kF, double kTolerance, double kSetpoint, CANSparkMax motor, RelativeEncoder encoder){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.kSetpoint = kSetpoint;
        this.motor = motor;
        this.encoder = encoder;
    }

    @Override
    public void initialize() {
        pid = new PIDController(kP, kI, kD, kF);
    }

    @Override
    public void execute() {
        motor.set(pid.calculate(encoder.getPosition(), kSetpoint));
    }
}
