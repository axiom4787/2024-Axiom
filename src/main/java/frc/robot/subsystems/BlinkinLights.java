// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ColorStates;


public class BlinkinLights extends SubsystemBase {
  private final Spark m_blinkin;
  private HashMap<ColorStates, Double> color_hashMap;
  

  public BlinkinLights() {
    m_blinkin = new Spark(0);

    // hasmap between color states and their color mGroundIntake,
    // mHumanIntake,
    // mShooter,
    // mdefault,
    // should be easy to add more colors and determing which color to set

    color_hashMap = new HashMap<ColorStates, Double>() {
      {
        put(ColorStates.mGroundIntake, 0.91); // violet
        put(ColorStates.mHumanIntake, 0.89); // blue violet
        put(ColorStates.mShooter, 0.53); // light chase
        put(ColorStates.mdefault, 0.57); // hotpink
      }
    };
  }

  public void set_color(ColorStates color) {
    m_blinkin.set(color_hashMap.get(color));
  }

  @Override
  public void periodic() {
    //set default
    // m_blinkin.set(0.41); gradient
    m_blinkin.set(0.45);
    // set_color(ColorStates.mShooter);
    // print("periodic - blinkin");
  }
}