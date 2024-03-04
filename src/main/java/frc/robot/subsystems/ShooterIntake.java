package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmIntakeShooter;

import java.util.concurrent.TimeUnit;

import java.util.concurrent.TimeUnit;

import com.revrobotics.CANSparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;


public class ShooterIntake extends SubsystemBase {
  private final CANSparkMax topMotor; //neo
  private final CANSparkMax bottomMotor; //neo 550

  private final RelativeEncoder topEncoder; //encoder for top 
  private final RelativeEncoder bottomEncoder; //encoder for bottom

  private String state = "off";

  XboxController driverXbox = new XboxController(0);

  public ShooterIntake() {
    topMotor = new CANSparkMax(Constants.ArmIntakeShooter.TOP_MOTORID, CANSparkMax.MotorType.kBrushless);
    bottomMotor = new CANSparkMax(Constants.ArmIntakeShooter.BOTTOM_MOTORID, CANSparkMax.MotorType.kBrushless);

    topMotor.setIdleMode(IdleMode.kBrake);

    topMotor.restoreFactoryDefaults();
    bottomMotor.restoreFactoryDefaults();

    topMotor.setInverted(false); 
    bottomMotor.setInverted(true); 

    // topMotor.follow(bottomMotor);

    topMotor.setSmartCurrentLimit(40);
    topMotor.setOpenLoopRampRate(0.05);
    bottomMotor.setSmartCurrentLimit(Constants.ArmIntakeShooter.BOTTOM_MOTOR_CURRENT_LIMIT);

    // topMotor.setIdleMode(TurretConstants.kShootMotorIdleMode);
    // bottomMotor.setIdleMode(TurretConstants.kRotateMotorIdleMode);

    topEncoder = topMotor.getEncoder();
    bottomEncoder = bottomMotor.getEncoder();
  }

    public void setState(String state){

      this.state = state;
    }
    
    private void simStateMachine() {
      if (state=="shoot") {
        topMotor.setInverted(true); 
        bottomMotor.setInverted(false);
        if (driverXbox.getLeftTriggerAxis() > 0.9) {
          bottomMotor.setVoltage(1.0);
          topMotor.setVoltage(0);
        }
        else if (driverXbox.getRightTriggerAxis() > 0.9) {
          bottomMotor.setVoltage(0);
          topMotor.setVoltage(1.0);
          // System.out.println("Speaker");
        }
        else {
          bottomMotor.setVoltage(0.0);
          topMotor.setVoltage(0.0);
          // System.out.println(stateLocation);
        }
      }
      else if (state=="intake") {
        topMotor.setInverted(false); 
        bottomMotor.setInverted(true); 
        bottomMotor.setVoltage(1.0);
        topMotor.setVoltage(1.0);
      }
      else if (state=="off") {
        bottomMotor.setVoltage(0.0);
        topMotor.setVoltage(0.0);
      }
      else {
        System.out.println("Invalid state");
      }
      // switch(stateDirection) {
      //   case "intake":
      //     topMotor.setInverted(false); 
      //     bottomMotor.setInverted(true); 
      //     bottomMotor.setVoltage(1.0);
      //     topMotor.setVoltage(1.0);
      //     System.out.println(stateDirection);
      //     break;
          
      //   case "shoot":
      //     topMotor.setInverted(true); 
      //     bottomMotor.setInverted(false); 
      //     bottomMotor.setVoltage(1.0);
      //     topMotor.setVoltage(1.0);
      //     System.out.println(stateDirection);
      //     switch(stateLocation) {
      //       System.out.println(stateLocation);
      //       case "amp":
      //         topMotor.setInverted(false); 
      //         bottomMotor.setInverted(false); 
      //         bottomMotor.setVoltage(1.0);
      //         topMotor.setVoltage(0);
      //         System.out.println(stateLocation);
      //         break;
      //       case "speaker":
      //         topMotor.setInverted(true); 
      //         bottomMotor.setInverted(true); 
      //         bottomMotor.setVoltage(0);
      //         topMotor.setVoltage(1.0);
      //         System.out.println(stateLocation);
      //         break;
      //       case "off":
      //         bottomMotor.setVoltage(0.0);
      //         topMotor.setVoltage(0.0);
      //         System.out.println(stateLocation);
      //         break;
      //     }

      //     break;
      //   case "off":
      //     bottomMotor.setVoltage(0.0);
      //     topMotor.setVoltage(0.0);
      //     System.out.println(stateDirection);
      //     break;
      // }
    }

    private void stateMachine() {
      if (state=="shoot") {
        topMotor.setInverted(true); 
        bottomMotor.setInverted(false);
        if (driverXbox.getLeftTriggerAxis() > 0.9) {
          bottomMotor.set(1.0);
          topMotor.set(0);

        }
        else if (driverXbox.getRightTriggerAxis() > 0.9) {
          bottomMotor.set(0);
          topMotor.set(1.0);
          // System.out.println("shooter spin");

        }
        else {
          bottomMotor.set(0.0);
          topMotor.set(0.0);
          // System.out.println(stateLocation);
        }

      }
      else if (state=="intake") {
        topMotor.setInverted(false); 
        bottomMotor.setInverted(true); 
        bottomMotor.set(1.0);
        topMotor.set(1.0);

      }
      else if (state=="off") {
        bottomMotor.set(0.0);
        topMotor.set(0.0);

      }
      else {

      }
    }

    @Override
    public void periodic() {
      // simStateMachine(); 
      stateMachine(); 
      System.out.println("motor output" + topMotor.getBusVoltage());
      
      // System.out.println(state);
      // topMotor.set(1);
    }
  
}
