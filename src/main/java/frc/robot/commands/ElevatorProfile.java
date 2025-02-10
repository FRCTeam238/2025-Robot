package frc.robot.commands;

import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MotionProfile;
import frc.robot.subsystems.Elevator;

public class ElevatorProfile extends Command {
  private final Elevator elevator = Elevator.getInstance();
  private final MotionProfile.State goal;
  private final MotionProfile.MotionConstraints constraints;
  private MotionProfile profile;
  private MotionProfile.State currentState;
  private boolean bypass;

  public ElevatorProfile(MotionProfile.State goal, String name) {
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.elevator);
    this.goal = goal;
    constraints =
        new MotionProfile.MotionConstraints(
            maxElevatorJerk, maxAccel, maxVelocity, velocityTolerance);
    elevator.setCommand(name);
  }

  @Override
  public void initialize() {

    currentState = new MotionProfile.State(elevator.getPosition(), elevator.getVelocity());

    profile = new MotionProfile(goal, currentState, constraints, MotionProfile.ProfileType.AUTO);
    elevator.setCommand(getName());
    bypass = onTarget();  //if already on target, skip
  }

  @Override
  public void execute() {
   
    MotionProfile.State sample = profile.sample();
    elevator.setDesiredState(sample);
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
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
