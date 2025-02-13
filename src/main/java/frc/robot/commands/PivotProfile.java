// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.PivotConstants.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MotionProfile;
import frc.robot.subsystems.Pivot;
import frc.robot.MotionProfile.MotionConstraints;
import frc.robot.MotionProfile.ProfileType;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotProfile extends Command {

  // to start, we want to make a command that takes a mechanism position and go to
  // it

  Pivot pivot;
  MotionProfile.State goal;

  MotionProfile.State currentState;
  MotionProfile profile;
  MotionProfile.MotionConstraints constraints = new MotionConstraints(maxJerk, maxAccel, maxVelocity,
      velocityTolerance);

  /** Creates a new PivotProfile. */
  public PivotProfile(MotionProfile.State goal, String name) {
    pivot = Pivot.getInstance();
    addRequirements(pivot);
    // Use addRequirements() here to declare subsystem dependencies.
    this.goal = goal;
    pivot.setCommand(name);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = new MotionProfile.State(pivot.getPosition(), pivot.getVelocity());
    profile = new MotionProfile(goal, currentState, constraints, ProfileType.SCURVE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: check if there are illegal positions we have to watch out for
    MotionProfile.State sample = profile.sample();
    pivot.setDesiredState(sample);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.setCommand("None");
    // pivot.holdPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTarget() && profile.isFinished();
  }

  public boolean onTarget() {
    return Math.abs(pivot.getVelocity() - goal.velocity) <= velocityMaxError
        && Math.abs(pivot.getPosition() - goal.position) <= positionMaxError;
  }
}
