package frc.robot.commands;

import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MotionProfile;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class ElevatorProfile extends Command {
  private final Elevator elevator = Elevator.getInstance();
  private final MotionProfile.State goal;
  private final MotionProfile.MotionConstraints constraints;
  private MotionProfile profile;
  private MotionProfile.State currentState;
  private boolean bypass;

  private String name;

  public ElevatorProfile(MotionProfile.State goal, String name) {
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.elevator);
    this.name = name;
    this.goal = goal;
    constraints = new MotionProfile.MotionConstraints(
        maxElevatorJerk, maxAccel, maxVelocity, velocityTolerance);
  }

  @Override
  public void initialize() {
    elevator.setCommand(name);

    currentState = new MotionProfile.State(elevator.getPosition(), elevator.getVelocity());

    profile = new MotionProfile(goal, currentState, constraints, MotionProfile.ProfileType.AUTO);
    bypass = onTarget(); // if already on target, skip
  }

  @Override
  public void execute() {
    if (goal.position < ElevatorConstants.dangerZone && Wrist.getInstance().getPosition() > WristConstants.dangerZone) {
      // wrist is in danger of hitting elevator, don't drop yet
    } else {
      MotionProfile.State sample = profile.sample();
      elevator.setDesiredState(sample);
    }
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run
    // execute()
    return bypass || (onTarget() && profile.isFinished());
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setCommand("None");
  }

  public boolean onTarget() {
    return Math.abs(elevator.getVelocity() - goal.velocity) <= velocityMaxError
        && Math.abs(elevator.getPosition() - goal.position) <= positionMaxError;
  }

}
