package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TwoBallAuto extends SequentialCommandGroup {

    public TwoBallAuto(DriveSubsystem drive, VisionSubsystem vision, ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubSystem intake) {

        addCommands(
            new RunCommand(() -> {
            drive.setChassisSpeeds(0, Constants.DRIVE_SPEED_AUTO, 0, false);
            intake.intakeBall();
        }, drive).withTimeout(2.5),
        new InstantCommand(drive::stop, drive),
        new ShooterTeleop(shooter,indexer,vision,drive));
    }
    
}
