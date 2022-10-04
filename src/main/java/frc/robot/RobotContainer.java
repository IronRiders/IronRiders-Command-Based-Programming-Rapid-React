package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    public final VisionSubsystem vision = new VisionSubsystem();
    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final DriveSubsystem drive = new DriveSubsystem();
    public final IndexerSubsystem indexer = new IndexerSubsystem();
    public final IntakeSubSystem intake = new IntakeSubSystem();
    public final ClimberSubsystem climber = new ClimberSubsystem();

    public static GenericHID controller = new GenericHID(0);

    AutoCommandFactory autocmdFactory = new AutoCommandFactory(shooter, drive, intake, indexer, vision);

    public RobotContainer() {

        SmartDashboard.putNumber("Shooter Offset RPM", 0);

        drive.setDefaultCommand(new RunCommand(() -> drive.setChassisSpeeds(controller.getRawAxis(0),
                controller.getRawAxis(1), controller.getRawAxis(3), true), drive));

        new JoystickButton(controller, 1)
                .whenHeld(new ShooterTeleop(shooter, indexer, vision, drive));

        new JoystickButton(controller, 2)
                .whenHeld(new StartEndCommand(intake::intakeBall, intake::stop, intake));

        new JoystickButton(controller, 3).whenPressed(new InstantCommand(drive::invertDrive, drive));

        new JoystickButton(controller, 4)
                .whenHeld(new StartEndCommand(climber::lower, climber::stop, climber));

        new JoystickButton(controller, 5)
                .whenHeld(CommandFactory.runIndexerCommand(indexer));

        new JoystickButton(controller, 9)
                .whenHeld(new StartEndCommand(intake::startDeployment, intake::finishDeployment,
                        intake));

        new JoystickButton(controller, 11)
                .whenHeld(new StartEndCommand(intake::spitOutBall, intake::stop, intake));

        new JoystickButton(controller, 12)
                .whenHeld(new StartEndCommand(climber::raise, climber::stop, climber));
    }

    public Command getAutonomousCommand() {
        return autocmdFactory.testAuto();
    }
}
