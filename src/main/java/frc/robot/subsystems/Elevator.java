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

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotionProfile;
import static frc.robot.Constants.ElevatorConstants.*;

@Logged
public class Elevator extends SubsystemBase {
  TalonFX leftMotor = new TalonFX(6);
  TalonFX rightMotor = new TalonFX(7);

  PositionVoltage elevatorVoltage = new PositionVoltage(0);

  String command = "None";

  @NotLogged private static Elevator singleton;

  ArmFeedforward ff;
  MotionProfile.State desiredState = new MotionProfile.State(0);

  /** Creates a new Elevator. */
  private Elevator() {
    ff = new ArmFeedforward(kS, kG, kV);

    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    leftConfig.Slot0.kP = kP;
    leftConfig.Slot0.kI = kI;
    leftConfig.Slot0.kD = kD;
    leftConfig.Slot0.kV = kV;
    leftConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;
    leftConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
    leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftConfig.Feedback.SensorToMechanismRatio = 1. / conversionFactor; // native unit is rotations, this converts to
                                                                         // inches
    leftMotor.getConfigurator().apply(leftConfig);

    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    rightConfig.Slot0.kP = kP;
    rightConfig.Slot0.kI = kI;
    rightConfig.Slot0.kD = kD;
    rightConfig.Slot0.kV = kV;
    rightConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;
    rightConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimit;
    rightConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    rightConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightMotor.getConfigurator().apply(rightConfig);
    rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

  }

  public static Elevator getInstance() {
    if (singleton == null)
      singleton = new Elevator();
    return singleton;
  }

  public void setDesiredState(MotionProfile.State state) {
    desiredState = state;
    //elevator runs perpendicular to arm so offset position for FF by 90 degrees
    double feed = ff.calculate(Units.degreesToRadians(90 - Pivot.getInstance().getPosition()), state.velocity,
        state.acceleration);

    elevatorVoltage.withFeedForward(feed).withPosition(state.position);
    leftMotor.setControl(elevatorVoltage);

  }

  /**
   * set PID to remain at the current position at zero velocity
   */
  public void holdPosition() {
    leftMotor.setControl(new PositionVoltage(getPosition()));
  }

  public Command holdPositionCommand() {
    return new RunCommand(() -> {
      holdPosition();
    }, getInstance());
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

  public void set(double speed) {
    leftMotor.set(speed);

  }

  /**
   * 
   * @return units in inches
   */
  public double getPosition() {

    return leftMotor.getPosition().getValueAsDouble();
  }

  public double getVelocity() {
    return leftMotor.getVelocity().getValue().baseUnitMagnitude();
  }

  public void setCommand(String name) {
    command = name;
  }
}
