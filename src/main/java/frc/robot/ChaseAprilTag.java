package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.MecanumPathFollower;
import frc.robot.subsystems.DriveSubsystem;

public class ChaseAprilTag {

    DriveSubsystem drive;
    double lastTime = 0;

    public ChaseAprilTag(DriveSubsystem drive) {
        this.drive = drive;
    }

    public void Update(Pose2d robotPose, Pose3d tagPose) {
        double now = Timer.getFPGATimestamp();
        if ((now - lastTime) > 0.1) {

            PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                    new PathConstraints(2, 2),
                    new PathPoint(new Translation2d(robotPose.getX(), robotPose.getY()), new Rotation2d (drive.getVelocity().vxMetersPerSecond, drive.getVelocity().vyMetersPerSecond)),
                    new PathPoint(new Translation2d(tagPose.getX(), tagPose.getY()), new Rotation2d(0, 0)));

            new MecanumPathFollower(trajectory, drive).schedule();

            lastTime = now;
        }
    }
}
