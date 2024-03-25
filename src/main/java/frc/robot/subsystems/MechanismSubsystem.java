package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

//import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.Constants.MechState;

public class MechanismSubsystem extends SubsystemBase {
    private final CANSparkMax m_frontShooter, m_backShooter;
    private final CANSparkMax m_indexer;
    private final CANSparkMax m_roller;
    private final CANSparkMax m_groundIntake;
    private MechState m_shooterState, m_rollerState, m_groundIntakeState;

    private final Timer m_indexerStartTimer = new Timer();
    

    public MechanismSubsystem()
    {
        m_frontShooter = new CANSparkMax(ShooterConstants.kFrontCanId, MotorType.kBrushless);
        m_backShooter = new CANSparkMax(ShooterConstants.kBackCanId, MotorType.kBrushless);
        m_indexer = new CANSparkMax(ShooterConstants.kIndexerCanId, MotorType.kBrushed);
        m_roller = new CANSparkMax(RollerConstants.kRollerCanId, MotorType.kBrushless);
        m_groundIntake = new CANSparkMax(GroundIntakeConstants.kGroundIntakeCanId, MotorType.kBrushless);
        m_frontShooter.setSmartCurrentLimit(30);
        m_backShooter.setSmartCurrentLimit(30);
        m_indexer.setSmartCurrentLimit(20);
        m_roller.setSmartCurrentLimit(30);
        m_groundIntake.setSmartCurrentLimit(30);
        m_frontShooter.setIdleMode(IdleMode.kCoast);
        m_backShooter.setIdleMode(IdleMode.kCoast);
        m_indexer.setIdleMode(IdleMode.kCoast);
        m_roller.setIdleMode(IdleMode.kCoast);
        m_groundIntake.setIdleMode(IdleMode.kCoast);
        m_shooterState = m_rollerState = m_groundIntakeState = MechState.mOff;

    }   

    @Override
    public void periodic()
    {
        switch (m_shooterState)
        {
            case mIntake:
                m_frontShooter.set(ShooterConstants.kShooterIntakeSpeed);
                m_backShooter.set(ShooterConstants.kShooterIntakeSpeed);
                m_indexer.set(ShooterConstants.kIndexerIntakeSpeed);
                break;
            case mShoot:
                m_frontShooter.set(ShooterConstants.kShooterLaunchSpeed);
                m_backShooter.set(ShooterConstants.kShooterLaunchSpeed);
                m_indexer.set(ShooterConstants.kIndexerLaunchSpeed);
                break;
            case mChargeShoot:
                m_frontShooter.set(ShooterConstants.kShooterLaunchSpeed);
                m_backShooter.set(ShooterConstants.kShooterLaunchSpeed);
                break;
            case mOff:
                m_frontShooter.set(0);
                m_backShooter.set(0);
                m_indexer.set(0);
        }
        switch (m_rollerState)
        {
            case mIntake:
                m_roller.set(RollerConstants.kRollerSpeed);
                break;
            case mShoot:
                m_roller.set(-RollerConstants.kRollerSpeed);
                break;
            case mChargeShoot:
                m_roller.set(-RollerConstants.kRollerSpeed);
                break;
            case mOff:
                m_roller.set(0);
        }
        switch (m_groundIntakeState)
        {
            case mIntake:
                m_groundIntake.set(GroundIntakeConstants.kIntakeSpeed);
                break;
            case mShoot:
                m_groundIntake.set(-GroundIntakeConstants.kIntakeSpeed);
                break;
            case mChargeShoot:
                break;
            case mOff:
                m_groundIntake.set(0);
                break;
        }

        // System.out.println("Shooter: " + m_shooterState + " Roller: " + m_rollerState);
    }

    public void setShooterState(MechState state)
    {
        if (state == MechState.mShoot && m_shooterState == MechState.mOff)
        {
            m_indexerStartTimer.reset();
            m_indexerStartTimer.start();
        }
        m_shooterState = state;
        System.out.println("Shooter state set to " + state);
    }

    public void setRollerState(MechState state)
    {
        m_rollerState = state;
        System.out.println("Roller state set to " + state);
    }

    public void setGroundIntakeState(MechState state)
    {
        m_groundIntakeState = state;
        System.out.println("Ground Intake state set to " + state);
    }

    public void setBothStates(MechState rollerState, MechState shooterState) {
        setRollerState(rollerState);
        setShooterState(shooterState);
    }

    public MechState getShooterState()
    {
        return m_shooterState;
    }

    public MechState getRollerState()
    {
        return m_rollerState;
    }

}