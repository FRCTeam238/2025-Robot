// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotionProfile;
import frc.robot.Robot;

import static frc.robot.Constants.ElevatorConstants.*;

public class Elevator extends SubsystemBase {
  TalonFX leftMotor = new TalonFX(0);
  TalonFX rightMotor = new TalonFX(0);

  PositionVoltage elevatorVoltage = new PositionVoltage(0);

  String command = "None";

  ArmFeedforward ff;

  /** Creates a new Elevator. */
  public Elevator() {

    ff = new ArmFeedforward(kS, kG, kV);

    TalonFXConfiguration leftConfig = new TalonFXConfiguration(); 
    leftConfig.Slot0.kP = kP;
    leftConfig.Slot0.kI = kI;
    leftConfig.Slot0.kD = kD;
    leftConfig.Slot0.kV = kV;
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
    leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftConfig.Feedback.SensorToMechanismRatio = 1/conversionFactor;
    leftMotor.getConfigurator().apply(leftConfig);
    
    TalonFXConfiguration rightConfig = new TalonFXConfiguration(); 
    rightConfig.Slot0.kP = kP;
    rightConfig.Slot0.kI = kI;
    rightConfig.Slot0.kD = kD;
    rightConfig.Slot0.kV = kV;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightMotor.getConfigurator().apply(rightConfig);
    rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

  }

  public void setDesiredState(MotionProfile.State state) {
    double feed = ff.calculate(Units.degreesToRadians(Robot.pivot.getPosition()), state.velocity, state.acceleration);

    elevatorVoltage.withFeedForward(feed).withPosition(state.position);
    leftMotor.setControl(elevatorVoltage);

  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }

  public void set(double speed){
    leftMotor.set(speed);
    
  } 

  /**
   * 
   * @return units in inches
   */
  public double getEncoderPosition() {

    return leftMotor.getPosition().getValueAsDouble();
  }

  public double getVelocity() {
    return leftMotor.getVelocity().getValue().baseUnitMagnitude();
  }

  public void setCommand(String name) {
    command = name;
  }
}
