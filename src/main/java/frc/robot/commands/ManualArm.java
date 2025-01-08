package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls;
import frc.robot.Robot;

public class ManualArm extends Command {
    
    public ManualArm() {
        addRequirements(Robot.arm);
    }
    
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        double speed = Controls.controller.getLeftY();
        Robot.arm.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.arm.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
