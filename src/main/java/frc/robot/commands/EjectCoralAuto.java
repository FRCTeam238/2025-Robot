// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.Auto;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EjectCoralAuto extends SequentialCommandGroup {
  /** Creates a new EjectCoralAuto. */
  @Auto
  public EjectCoralAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(new EjectCoral().withTimeout(1.5));
  }

}
