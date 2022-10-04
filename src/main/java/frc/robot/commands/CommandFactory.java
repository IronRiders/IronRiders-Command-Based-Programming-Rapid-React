package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class CommandFactory {
    public static SequentialCommandGroup runIndexerCommand(IndexerSubsystem indexer) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> indexer.extend(), indexer),
                new WaitCommand(1),
                new InstantCommand(() -> indexer.retract(), indexer));
    }
}