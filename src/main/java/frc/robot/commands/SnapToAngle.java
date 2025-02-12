// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls;
import frc.robot.autonomous.Auto;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.DriveConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SnapToAngle extends Command {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private double angle;
  private PIDController pid;
  /** Creates a new SnapToAngle. */
  
  @Auto(names = {"Angle (Degrees)"})
  public SnapToAngle(double degrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    angle = degrees;
    addRequirements(drivetrain);
    pid = new PIDController(kPAngular, kIAngular, kDAngular);
    pid.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] joyValues = Controls.getInstance().getSwerveJoystickValues();
    double robotAngle = drivetrain.getFieldRelativeOffset().getRadians();

    drivetrain.drive(joyValues[0], joyValues[1], pid.calculate(robotAngle, angle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
