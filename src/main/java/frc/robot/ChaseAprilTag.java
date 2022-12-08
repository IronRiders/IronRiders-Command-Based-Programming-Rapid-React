package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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

    public void Update(Pose3d tagPose) {
        double now = Timer.getFPGATimestamp();
        if ((now - lastTime) > 0.1) {

            var tagPose2d = tagPose.toPose2d();
            var chassisSpeeds = drive.getChasssisSpeeds();

            var initialHeading = new Rotation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
                    .plus(drive.getPose2d().getRotation());
            var initialTrl = new Translation2d(drive.getxController().getSetpoint(),
                    drive.getyController().getSetpoint());
            if (initialTrl.getDistance(drive.getPose2d().getTranslation()) > 0.5)
                initialTrl = drive.getPose2d().getTranslation();
            var initialHolRot = new Rotation2d(drive.getThetaController().getSetpoint());

            var finalPose = tagPose2d.plus(new Transform2d(new Translation2d(1, 0), new Rotation2d()));
            var finalHeading = tagPose2d.getRotation().plus(new Rotation2d(Math.PI));
            var finalHolRot = finalPose.getRotation();

            var initialPoint = new PathPoint(initialTrl, initialHeading, initialHolRot,
                    Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond));
            var finalPoint = new PathPoint(finalPose.getTranslation(), finalHeading, finalHolRot);

            PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                    new PathConstraints(1, 1), initialPoint, finalPoint);

            new MecanumPathFollower(trajectory, drive).schedule();

            lastTime = now;
        }
    }
}
