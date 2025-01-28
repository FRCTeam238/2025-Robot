// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;
import java.lang.module.Configuration;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  TalonFX leftMotor = new TalonFX(0);
  TalonFX rightMotor = new TalonFX(0);

  /** Creates a new Elevator. */
  public Elevator() {
    TalonFXConfiguration leftConfig = new TalonFXConfiguration(); 
    leftConfig.Slot0.kP = 0;
    leftConfig.Slot0.kI = 0;
    leftConfig.Slot0.kD = 0;
    leftConfig.Slot0.kV = 0;
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.CurrentLimits.StatorCurrentLimit = 0;
    leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftMotor.getConfigurator().apply(leftConfig);
    
    TalonFXConfiguration rightConfig = new TalonFXConfiguration(); 
    rightConfig.Slot0.kP = 0;
    rightConfig.Slot0.kI = 0;
    rightConfig.Slot0.kD = 0;
    rightConfig.Slot0.kV = 0;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.CurrentLimits.StatorCurrentLimit = 0;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightMotor.getConfigurator().apply(rightConfig);
    rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

   }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
  public void set(double speed){
    leftMotor.set(speed);
    
  } 
}
