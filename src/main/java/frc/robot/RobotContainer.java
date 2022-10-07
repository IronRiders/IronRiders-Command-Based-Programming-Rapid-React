package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    public final VisionSubsystem vision = new VisionSubsystem();
    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final DriveSubsystem drive = new DriveSubsystem();
    public final IndexerSubsystem indexer = new IndexerSubsystem();
    public final IntakeSubSystem intake = new IntakeSubSystem();
    public final ClimberSubsystem climber = new ClimberSubsystem();

    public static GenericHID controller = new GenericHID(0);

    public RobotContainer() {
        // Drive
        drive.setDefaultCommand(new RunCommand(() -> drive.updateSpeed(joystickResponse(controller.getRawAxis(0)),
                joystickResponse(controller.getRawAxis(1)), joystickResponse(controller.getRawAxis(3)), true), drive));

        shooter.setDefaultCommand(new RunCommand(() -> shooter.shoot(50), shooter));

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
            return CommandFactory.TwoBallAuto(drive, vision, shooter, indexer, intake);
    }

    // This adds a deadzone and nonlinear response to the joystick axis
    private double joystickResponse(double raw) {
        double deadband = SmartDashboard.getNumber("deadband", Constants.DEADBAND);
        double deadbanded = 0.0;
        if (raw > deadband) {
            deadbanded = (raw - deadband) / (1 - deadband);
        } else if (raw < -deadband) {
            deadbanded = (raw + deadband) / (1 - deadband);
        }
        double exponent = SmartDashboard.getNumber("exponent", Constants.EXPONENT) + 1;
        return Math.pow(Math.abs(deadbanded), exponent) * Math.signum(deadbanded);
    }
}
