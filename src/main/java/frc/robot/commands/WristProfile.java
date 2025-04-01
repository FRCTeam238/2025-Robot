// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.WristConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.MotionProfile;
import frc.robot.MotionProfile.MotionConstraints;
import frc.robot.MotionProfile.ProfileType;
import frc.robot.autonomous.Auto;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristProfile extends Command {

  Wrist wrist;

  MotionProfile.State goal;
  MotionProfile profile;
  String name;

  private MotionProfile.State current;

  /** Creates a new WristProfile. */
  public WristProfile(MotionProfile.State goal, String name) {
    this.goal = goal;
    this.name = name;
    wrist = Wrist.getInstance();
    addRequirements(wrist);
  }

  @Auto
  public WristProfile() {
    this(new MotionProfile.State(WristConstants.deepCage), "DeepCage");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.setCommand(name);

    current = new MotionProfile.State(wrist.getPosition(), wrist.getVelocity());
    profile = new MotionProfile(goal, current, new MotionConstraints(maxJerk, maxAccel, maxVelocity, velocityTolerance),
        ProfileType.SCURVE);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (goal.position > WristConstants.dangerZone
        && Elevator.getInstance().getPosition() < ElevatorConstants.dangerZone) {
      // wrist is in danger of hitting elevator, don't move wrist until elevator gets
      // high enough
    } else {
      MotionProfile.State sample = profile.sample();
      wrist.setDesiredState(sample);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setCommand("None");
    // wrist.holdPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(wrist.getPosition() - goal.position) <= maxPositionTolerance
        && Math.abs(wrist.getVelocity() - goal.velocity) <= maxVelocityTolerance;
  }
}
