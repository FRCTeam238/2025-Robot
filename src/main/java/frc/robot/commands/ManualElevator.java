// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualElevator extends Command {

  /** Creates a new ManualElevator. */
  public ManualElevator() {
    addRequirements(Elevator.getInstance());
    Elevator.getInstance().setCommand("Manual");
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = Controls.getInstance().getOperatorLeftStickY();
    Elevator.getInstance().set(speed * -0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Elevator.getInstance().set(0);
    // Elevator.getInstance().holdPosition();
    Elevator.getInstance().setCommand("None");
    // Elevator.getInstance().setDesiredState(new Elevator.getInstance().getPosition());;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
