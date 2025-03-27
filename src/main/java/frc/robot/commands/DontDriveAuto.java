package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.Auto;
import frc.robot.subsystems.Drivetrain;

public class DontDriveAuto extends Command {

    Drivetrain d;
    
    @Auto
    public DontDriveAuto() {
        d = Drivetrain.getInstance();
        addRequirements(d);
    }

    @Override
    public void initialize() {
        d.setCommand("Dont Drive");
    }

    @Override
    public void execute() {
        d.driveFromJoysticks(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        d.setCommand("None");
    }
}
