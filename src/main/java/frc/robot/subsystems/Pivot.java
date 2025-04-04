// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.PivotConstants.*;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotionProfile;

@Logged
public class Pivot extends SubsystemBase {

    SparkMax pivotLeader;
    SparkMax pivotFollower;

    SparkAbsoluteEncoder encoder;
    ArmFeedforward ff;

    String commandName = "None";

    @NotLogged private static Pivot singleton;

    MotionProfile.State desiredState = new MotionProfile.State(0);
    double feed = 0;

    /** Creates a new Pivot. */
    private Pivot() {
        pivotLeader = new SparkMax(2, MotorType.kBrushless);
        pivotFollower = new SparkMax(17, MotorType.kBrushless);

        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        ff = new ArmFeedforward(kS, kG, kV);

        encoder = pivotLeader.getAbsoluteEncoder();

        // TODO: finish configs
        followerConfig.follow(pivotLeader, true).closedLoop.p(kP).i(kI).d(kD);
        followerConfig.idleMode(IdleMode.kBrake);

        leaderConfig.closedLoop.p(kP).i(kI).d(kD);
        leaderConfig.idleMode(IdleMode.kBrake);
        leaderConfig.inverted(true);
        leaderConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).positionWrappingEnabled(true);
        leaderConfig.absoluteEncoder
            .positionConversionFactor(360)
            .velocityConversionFactor(360)
            .inverted(true);
        leaderConfig.softLimit
            .forwardSoftLimit(0)
            .reverseSoftLimit(0)
            .reverseSoftLimitEnabled(false)
            .forwardSoftLimitEnabled(false);
        pivotFollower.configure(
            followerConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );
        pivotLeader.configure(
            leaderConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public static Pivot getInstance() {
        if (singleton == null) singleton = new Pivot();
        return singleton;
    }

    /**
     * sets the motor PID position to the current position with a speed of zero
     */
    public void holdPosition() {
        setDesiredState(new MotionProfile.State(getPosition()));
    }

    public Command holdPositionCommand() {
        return new RunCommand(
            () -> {
                holdPosition();
            },
            getInstance()
        );
    }

    // SETTERS

    public void setSpeed(double speed) {
        pivotLeader.set(speed);
    }

    public void stop() {
        pivotLeader.set(0);
      }

    public void setCommand(String name) {
        commandName = name;
    }

    /**
     * sets the position, velocity, and acceleration that we want the robot to
     * achieve
     *
     * @param state - the state of the pivot that we want to be in
     */
    public void setDesiredState(MotionProfile.State state) {
        desiredState = state;
        feed = ff.calculate(
            Units.degreesToRadians(state.position),
            Units.degreesToRadians(state.velocity),
            Units.degreesToRadians(state.acceleration)
        );

        pivotLeader
            .getClosedLoopController()
            .setReference(
                state.position,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                feed
            );
    }

    // GETTERS

    public String getCommand() {
        return commandName;
    }

    /**
     *
     * @return position in degrees
     */
    public double getPosition() {
        return encoder.getPosition();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }
}
