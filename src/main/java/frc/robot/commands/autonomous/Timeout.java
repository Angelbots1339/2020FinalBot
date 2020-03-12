package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

/**
 * Timeout
 */
public class Timeout extends ParallelRaceGroup {
    public Timeout(Command command, double timeout) {
        super(command, new Wait(timeout));
    }
}