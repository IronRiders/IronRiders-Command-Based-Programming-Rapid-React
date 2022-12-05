package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ChaseAprilTag extends CommandBase {

    private final DriveSubsystem drive;
    Trajectory trajectory;
    MecanumPathFollower mecanumPathFollower;

    public ChaseAprilTag(Trajectory trajectory, DriveSubsystem drive) {
        this.drive = drive;
        this.trajectory = trajectory;
        mecanumPathFollower = new MecanumPathFollower((PathPlannerTrajectory) trajectory, drive);

        addRequirements(drive);
    }
}
