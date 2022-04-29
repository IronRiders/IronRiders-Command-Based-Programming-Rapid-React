package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class ShooterTeleop extends SequentialCommandGroup {
    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;

    public ShooterTeleop(ShooterSubsystem shooter, IndexerSubsystem indexer, VisionSubsystem vision, DriveSubsystem drive) {
        this.shooter = shooter;
        this.indexer = indexer;
        if (vision.estimateDistance() < 142 && vision.estimateDistance() > 138) {
        addCommands(
            new RunCommand(() -> {
                drive.updateSpeed(0, 0, vision.steeringAssist(), false);
                shooter.shoot(2075);
            }, drive, shooter, vision).withTimeout(1.5),
            new InstantCommand(drive::stop, drive),
            CommandFactory.runIndexerCommand(indexer));
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        indexer.retract();
    }
}
