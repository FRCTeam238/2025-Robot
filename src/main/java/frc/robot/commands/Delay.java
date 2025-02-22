package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autonomous.Auto;

public class Delay extends SequentialCommandGroup {
    @Auto(names = {"Time to wait (Seconds)"})
    public Delay(double seconds) {
        addCommands(new WaitCommand(seconds));
    }
}
