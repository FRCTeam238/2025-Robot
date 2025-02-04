// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;

import static frc.robot.Constants.CoralIntakeConstants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
  /** Creates a new CoralIntake. */

  TalonFX coralMotor = new TalonFX(0);
  DigitalInput coralSensor = new DigitalInput(0);
  LaserCan lc;
  public CoralIntake() {
    TalonFXConfiguration coralIntakeConfig = new TalonFXConfiguration();
    lc = new LaserCan(0);
    coralIntakeConfig.CurrentLimits.StatorCurrentLimit = currentLimit;
    coralIntakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    coralMotor.getConfigurator().apply(coralIntakeConfig);
    try {
      lc.setRangingMode(RangingMode.SHORT);
    } catch (ConfigurationFailedException e) {
     e.printStackTrace(); 
    }
  }

  public void setSpeed(double speed) {
    coralMotor.set(speed);
  }

  public boolean hasCoral() {
    return coralSensor.get();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
