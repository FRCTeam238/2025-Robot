// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotionProfile;

public class Pivot extends SubsystemBase {

  SparkMax pivotLeader;
  SparkMax pivotFollower;

  SparkAbsoluteEncoder encoder;
  ArmFeedforward ff;

  /** Creates a new Pivot. */
  public Pivot() {
    pivotLeader = new SparkMax(0, MotorType.kBrushless);
    pivotFollower = new SparkMax(0, MotorType.kBrushless);

    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();


    //TODO: make numbers real
    ff = new ArmFeedforward(0, 0, 0);

    encoder = pivotLeader.getAbsoluteEncoder();

    //TODO: finish configs
    followerConfig.follow(pivotLeader)
    .inverted(true)
    .closedLoop
      .p(0)
      .i(0)
      .d(0)
      .maxMotion
        .maxAcceleration(0)
        .maxVelocity(0);

    leaderConfig.follow(pivotLeader)
    .closedLoop
      .p(0)
      .i(0)
      .d(0)
      .maxMotion
        .maxAcceleration(0)
        .maxVelocity(0);

    pivotFollower.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    pivotLeader.configure(leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * sets the motor PID position to the current position with a speed of zero
   */ 
  public void holdPosition() {
    pivotLeader.getClosedLoopController().setReference(getVelocity(), ControlType.kPosition, ClosedLoopSlot.kSlot0, 0);
  }

  //SETTERS


  public void setSpeed(double speed) {
    pivotLeader.set(speed);
  }

  /**
   * sets the position, velocity, and acceleration that we want the robot to achieve 
   * 
   * @param state - the state of the pivot that we want to be in
   */
  public void setDesiredState(MotionProfile.State state) {
    double feed = ff.calculate(state.position, state.velocity, state.acceleration);

    pivotLeader.getClosedLoopController().setReference(state.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feed);
  }

  //GETTERS

  public double getPosition() {
    return encoder.getPosition();
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }


}
