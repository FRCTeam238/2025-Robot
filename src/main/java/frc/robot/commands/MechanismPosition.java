// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ElevatorConstants.L1;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CoralMechanismState;
import frc.robot.MotionProfile;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.ElevatorConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MechanismPosition extends ParallelCommandGroup {

  CoralMechanismState mechState;

  /** Creates a new MechanismPosition. */
  public MechanismPosition(CoralMechanismState state) {
    
    mechState = state;

    switch (mechState) {
      case L1 -> {
        addCommands(
            new ElevatorProfile(new MotionProfile.State(ElevatorConstants.L1), "L1"),
            new PivotProfile(new MotionProfile.State(PivotConstants.L1)),
            new WristProfile(new MotionProfile.State(WristConstants.L1))
        );
      }
      case L2 -> {
        addCommands(
            new ElevatorProfile(new MotionProfile.State(ElevatorConstants.L2), "L2"),
            new PivotProfile(new MotionProfile.State(PivotConstants.L2)),
            new WristProfile(new MotionProfile.State(WristConstants.L2))
        );
      }
      case L3 -> {
        addCommands(
            new ElevatorProfile(new MotionProfile.State(ElevatorConstants.L3), "L3"),
            new PivotProfile(new MotionProfile.State(PivotConstants.L3)),
            new WristProfile(new MotionProfile.State(WristConstants.L3))
        );
      }
      case L4 -> {
        addCommands(
            new ElevatorProfile(new MotionProfile.State(ElevatorConstants.L4), "L4"),
            new PivotProfile(new MotionProfile.State(PivotConstants.L4)),
            new WristProfile(new MotionProfile.State(WristConstants.L4))
        );
      }
      case CoralStation -> {
        addCommands(
            new ElevatorProfile(new MotionProfile.State(ElevatorConstants.coralStation), "L1"),
            new PivotProfile(new MotionProfile.State(PivotConstants.coralStation)),
            new WristProfile(new MotionProfile.State(WristConstants.coralStation))
        );
      }
      case DeepCage -> {
        addCommands(
            new ElevatorProfile(new MotionProfile.State(ElevatorConstants.deepCage), "L1"),
            new PivotProfile(new MotionProfile.State(PivotConstants.deepCage)),
            new WristProfile(new MotionProfile.State(WristConstants.deepCage))
        );
      }
      case ShallowCage -> {
        addCommands(
            new ElevatorProfile(new MotionProfile.State(ElevatorConstants.shallowCage), "ShallowCage"),
            new PivotProfile(new MotionProfile.State(PivotConstants.shallowCage)),
            new WristProfile(new MotionProfile.State(WristConstants.shallowCage))
        );
      }
      case Stow -> {
        addCommands(
            new ElevatorProfile(new MotionProfile.State(ElevatorConstants.stow), "Stow"),
            new PivotProfile(new MotionProfile.State(PivotConstants.stow)),
            new WristProfile(new MotionProfile.State(WristConstants.stow))
        );
      }
    }
  }
}
