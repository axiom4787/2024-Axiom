package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorTestConstants;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

public class MotorTest extends SubsystemBase{
    public final CANSparkMax testMotor; //neo

    public MotorTest() {
        testMotor = new CANSparkMax(MotorTestConstants.MOTOR_ID, CANSparkMax.MotorType.kBrushless);
        testMotor.setSmartCurrentLimit(40);
        testMotor.setIdleMode(IdleMode.kBrake);
    }
    public void move(double speed){
        testMotor.set(speed);
    }
}
