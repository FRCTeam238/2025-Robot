// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

import static frc.robot.Constants.CoralIntakeConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
  /** Creates a new CoralIntake. */

  TalonFX coralMotor = new TalonFX(0);
  LaserCan lc;

  private static CoralIntake singleton;

  private CoralIntake() {
    TalonFXConfiguration coralIntakeConfig = new TalonFXConfiguration();
    lc = new LaserCan(0);
    coralIntakeConfig.CurrentLimits.StatorCurrentLimit = currentLimit;
    coralIntakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    coralMotor.getConfigurator().apply(coralIntakeConfig);
    try {
      lc.setRangingMode(RangingMode.SHORT);
      lc.setRegionOfInterest(new RegionOfInterest(ROIx, ROIy, ROIw, ROIh));
      lc.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
    } catch (ConfigurationFailedException e) {
     e.printStackTrace(); 
    }
  }

  public static CoralIntake getInstance() {
    if (singleton == null) {
      singleton = new CoralIntake();
    }
    return singleton;
  }

  public void setSpeed(double speed) {
    coralMotor.set(speed);
  }

  public boolean hasCoral() {
    return sensorDistance >= lc.getMeasurement().distance_mm;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
