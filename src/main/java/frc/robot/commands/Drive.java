package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

/** Drive */
public class Drive extends Command {

  Drivetrain drivetrain = Drivetrain.getInstance();

  public Drive() {
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.setCommand("Drive");
  }

  @Override
  public void execute() {
    double[] joyValues = Controls.getInstance().getSwerveJoystickValues();

    drivetrain.drive(
        joyValues[0] * maxVelocityMetersPerSec,
        joyValues[1] * maxVelocityMetersPerSec,
        joyValues[2] * maxAngularVelocityRadsPerSec);
  }
}
