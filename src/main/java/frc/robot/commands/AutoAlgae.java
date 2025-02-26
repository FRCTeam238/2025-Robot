// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AlgaeMechanismState;
import frc.robot.autonomous.Auto;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlgae extends SequentialCommandGroup {
  /** Creates a new AutoAlgae. */
  @Auto(names = {"Direction"})
  public AutoAlgae(AlgaeMechanismState state) {
    if (state == AlgaeMechanismState.Out) {
      addCommands(new AlgaeProfile(AlgaeMechanismState.Out).alongWith(new RunAlgaeIntake(false)).andThen(new AlgaeProfile(AlgaeMechanismState.Stow)));
    } else if (state == AlgaeMechanismState.Processor){
      addCommands(new AlgaeProfile(AlgaeMechanismState.Out).andThen(new RunAlgaeIntake(true).withTimeout(3)).andThen(new AlgaeProfile(AlgaeMechanismState.Stow)));
    } else {
      addCommands(new AlgaeProfile(AlgaeMechanismState.Stow));
    }
  }
}
