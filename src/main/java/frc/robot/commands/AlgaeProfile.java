// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MotionProfile;
import frc.robot.Constants.AlgaeMechanismState;
import frc.robot.subsystems.AlgaeIntake;
import static frc.robot.Constants.AlgaeIntakeConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeProfile extends Command {
  AlgaeIntake intake = AlgaeIntake.getInstance();


  MotionProfile.State goal;
  MotionProfile profile;
  AlgaeMechanismState desiredMechanismState;
  /** Creates a new ExtendAlgae. */
  public AlgaeProfile(AlgaeMechanismState state) {
    // Use addRequirements() here to declare subsystem dependencies.
    if (state == AlgaeMechanismState.Out) {
      goal = new MotionProfile.State(outPosition);
    } else {
      goal = new MotionProfile.State(inPosition);
    }
    addRequirements(intake);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    MotionProfile.State currentState = new MotionProfile.State(intake.getTurnPosition(), intake.getTurnSpeed());
    profile = new MotionProfile(goal, currentState, constraints, MotionProfile.ProfileType.AUTO);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setPosition(profile.sample());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTarget() && profile.isFinished();
  }

  //as an alternative, have it run until it hits the hardstop?
  public boolean onTarget() {
    return Math.abs(intake.getTurnSpeed() - goal.velocity) <= velocityMaxError
        && Math.abs(intake.getTurnPosition() - goal.position) <= positionMaxError;

  }
}
