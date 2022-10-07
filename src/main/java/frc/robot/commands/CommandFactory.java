package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class CommandFactory {
    public static SequentialCommandGroup runIndexerCommand(IndexerSubsystem indexer) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> indexer.extend(), indexer),
                new WaitCommand(0.5),
                new InstantCommand(() -> indexer.retract(), indexer));
    }

    public static SequentialCommandGroup TwoBallAuto(DriveSubsystem drive, VisionSubsystem vision, ShooterSubsystem shooter, IndexerSubsystem indexer,
    IntakeSubSystem intake) {
        return new SequentialCommandGroup(
            new RunCommand(() -> {
                intake.startDeployment();
                drive.updateSpeed(0, Constants.DRIVE_SPEED_AUTO, 0, false);
                intake.intakeBall();
            }, drive).withTimeout(3),
            new InstantCommand(drive::stop, drive),
            new ShooterTeleop(shooter, indexer, vision, drive),
            new WaitCommand(3.0),
            new ShooterTeleop(shooter, indexer, vision, drive),
            new InstantCommand(intake::stop,intake));

    }
}
