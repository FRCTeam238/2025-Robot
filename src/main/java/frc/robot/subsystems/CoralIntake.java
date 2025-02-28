// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

import static frc.robot.Constants.CoralIntakeConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class CoralIntake extends SubsystemBase {
  /** Creates a new CoralIntake. */

  TalonFX coralMotor = new TalonFX(5);
  @NotLogged LaserCan lc;
  @NotLogged DigitalInput di;

  @NotLogged private static CoralIntake singleton;
  private boolean hadCoralLast = false;

  private CoralIntake() {
    di = new DigitalInput(0);
    TalonFXConfiguration coralIntakeConfig = new TalonFXConfiguration();
    lc = new LaserCan(0);
    coralIntakeConfig.Slot0.kP = 0.2;
    coralIntakeConfig.Slot0.kV = 0.13;
    coralIntakeConfig.CurrentLimits.StatorCurrentLimit = currentLimit;
    coralIntakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    coralMotor.getConfigurator().apply(coralIntakeConfig);
    try {
      lc.setRangingMode(RangingMode.SHORT);
      // lc.setRegionOfInterest(new RegionOfInterest(ROIx, ROIy, ROIw, ROIh));
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
    coralMotor.setControl(new VelocityVoltage(speed));
  }

  public boolean hasCoral() {

    if (lc.getMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      hadCoralLast = sensorDistance >= lc.getMeasurement().distance_mm;
    }
    return hadCoralLast;
    // return !di.get();
  }
  public double getDistanceFromCoral() {
    // if (lc.getMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return lc.getMeasurement().distance_mm;
    // } 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
