package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ChaseAprilTag extends CommandBase {

    private final PhotonCamera camera;
    private final DriveSubsystem drive;
    private final Supplier<Pose3d> poseProvider;
    private static final int TAG_TO_CHASE = 0;
    private PhotonTrackedTarget lastTarget;

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(0, 0);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(0, 0);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = new TrapezoidProfile.Constraints(0, 0);
    private final ProfiledPIDController xController = new ProfiledPIDController(0, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(0, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(0, 0, 0, OMEGA_CONSTRATINTS);

    public ChaseAprilTag(PhotonCamera camera, DriveSubsystem drive, Supplier<Pose3d> poseProvider) {
        this.camera = camera;
        this.drive = drive;
        this.poseProvider = poseProvider;

        addRequirements(drive);
    }

    public void intialize() {
        var robotPose = poseProvider.get();
        omegaController.reset(robotPose.getRotation().getAngle());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    @Override
    public void execute() {
        var robotPose = poseProvider.get();
        var photonResults = camera.getLatestResult();

        if (photonResults.hasTargets()) {
            // finding the tag
            var targetOutput = photonResults.getTargets().stream().filter(t -> t.getFiducialId() == TAG_TO_CHASE)
                    .findFirst();
            if (targetOutput.isPresent()) {
                var target = targetOutput.get();
                if (!target.equals(lastTarget)) {

                }

            }
        }
    }
}
