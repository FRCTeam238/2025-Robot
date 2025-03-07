// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunAlgaeIntake extends Command {
  /** Creates a new RunAlgaeIntake. */
  private boolean reversed = false; 
  private AlgaeIntake intake = AlgaeIntake.getInstance();
  /**
   * pulls in by default
   * @param reversed
   */
  public RunAlgaeIntake(boolean reversed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.reversed = reversed;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!reversed) {
      intake.setRunSpeed(-0.5); //check if this is backwards
    } else {
      intake.setRunSpeed(0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setRunSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.isStallingOnAlgae() && !reversed;
  }
}
