// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.AlgaeIntakeConstants.*;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotionProfile;
import frc.robot.Constants.AlgaeMechanismState;

@Logged
public class AlgaeIntake extends SubsystemBase {

  @NotLogged
  SparkMax turn;
  @NotLogged
  TalonFX drive;

  @NotLogged
  ArmFeedforward ff;

  public AlgaeMechanismState state = AlgaeMechanismState.Stow;

  String command = "None";

  // @NotLogged
  MotionProfile.State desiredState = new MotionProfile.State(0);

  @NotLogged private static AlgaeIntake singleton;


  /** Creates a new AlgaeIntake. */
  public AlgaeIntake() {
    ff = new ArmFeedforward(kS, kG, kV);
    turn = new SparkMax(16, MotorType.kBrushless);
    drive = new TalonFX(3);

    SparkMaxConfig sConf = new SparkMaxConfig();
    sConf.closedLoop
     .p(kP)
     .i(kI)
     .d(kD);
    sConf.idleMode(IdleMode.kBrake);
    turn.configure(sConf, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    TalonFXConfiguration tConf = new TalonFXConfiguration();
    tConf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tConf.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    drive.getConfigurator().apply(tConf);
    
  }

  public static AlgaeIntake getInstance() {
    if (singleton == null) {
      singleton = new AlgaeIntake();
    }
    return singleton;
  }

  public void setCommand(String name) {
    command = name;
  }

  public String getCommand() {
    return command;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //this is what i think is the correct method, but it may not be?????
  public void setPosition(MotionProfile.State state) {
   desiredState = state;

   double feed = ff.calculate(Units.degreesToRadians(state.position), Units.degreesToRadians(state.velocity));

   turn.getClosedLoopController().setReference(state.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feed);
  }
  
  public void setRunSpeed(double speed) {
    drive.set(speed);
  }

  public void stopAll() {
    drive.set(0);
    turn.set(0);
  }

  public double getTurnPosition() {
    return turn.getEncoder().getPosition();
  }

  public double getTurnSpeed() {
    return turn.getEncoder().getVelocity();
  }

  public boolean isStallingOnAlgae() {
    return drive.getStatorCurrent().getValueAsDouble() > currentDetectionLimit;
  }
}
