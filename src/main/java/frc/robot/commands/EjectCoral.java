// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.CoralMechanismState;
import frc.robot.autonomous.Auto;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EjectCoral extends Command {
  private CoralIntake coralIntake = CoralIntake.getInstance();
  /** Creates a new EjectCoral. */
  @Auto
  public EjectCoral() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.coralState == CoralMechanismState.L1) {
      coralIntake.setSpeed(-.25);
    } else {
      coralIntake.setSpeed(.25);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralIntake.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !coralIntake.hasCoral();
  }
}
