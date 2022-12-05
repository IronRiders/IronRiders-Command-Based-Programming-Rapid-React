package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AprilTagFieldLayout;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
    private NetworkTable table;
    private PIDController pidController;
    public final PhotonCamera camera = new PhotonCamera("Camera Name");
    private AprilTagFieldLayout tagLayout;
    private List<Pose3d> targetPoses;
    private List<Pose3d> robotPoses;
    public PhotonPipelineResult previousPipelineResult = null;

    public static final List<Pose3d> allTargetPoses = List.of(
            new Pose3d(new Translation3d(-0.0035306, 7.578928199999999, .8858503999999999), (new Rotation3d(0, 0, 0))));

    public VisionSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        pidController = new PIDController(Constants.TURN_P, 0.0, 0.0);
        pidController.setSetpoint(0);
        pidController.setTolerance(Constants.TURN_TOLERANCE);

        // April Tag
        robotPoses = new ArrayList<>();
        targetPoses = new ArrayList<>();
        try {
            tagLayout = new AprilTagFieldLayout("AprilTags_RapidReact.json");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public double getXAngleOffset() {
        double tx = table.getEntry("tx").getDouble(0.0);
        return Double.isNaN(tx) ? 0.0 : tx;
    }

    public double getYAngleOffset() {
        double ty = table.getEntry("ty").getDouble(0.0);
        return Double.isNaN(ty) ? 0.0 : ty;
    }

    public boolean getHasTargets() {
        return table.getEntry("tv").getDouble(0.0) == 1.0;
    }

    public double estimateDistance() {
        double degrees = Constants.VISION_CAMERA_ANGLE + getYAngleOffset();
        // https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
        double distance = Constants.VISION_DELTA_HEIGHT / Math.tan(degrees * Math.PI / 180.0);
        return distance;
    }

    // Adjusts the distance between a vision target and the robot using ty and PID
    public double distanceAssist() {
        double distanceError = estimateDistance() - Constants.TARGET_DIST;
        if (!getHasTargets() || Math.abs(distanceError) < Constants.OK_DISTANCE) {
            return 0;
        }
        double adjustment = distanceError * Constants.KP_DIST;
        return Math.min(Constants.DIST_MAX_SPEED, Math.max(-Constants.DIST_MAX_SPEED, adjustment));
    }

    // Adjusts the angle facing a vision target using Limelight tx and PID
    public double steeringAssist() {
        double offset = getXAngleOffset() - Math.atan(12 / estimateDistance());
        if (!getHasTargets() || Math.abs(offset) < Constants.TURN_MIN_ANGLE) {
            return 0;
        }
        double adjustment = pidController.calculate(offset);
        adjustment = Math.min(Constants.TURN_MAX_SPEED, Math.max(-Constants.TURN_MAX_SPEED, adjustment));
        adjustment = pidController.atSetpoint() ? 0 : -adjustment;
        return adjustment;
    }

    public void aprilTag() {
        var result = camera.getLatestResult();
        var targets = result.getTargets();

        for (var target : targets) {
            var tag = tagLayout.getTagPose(target.getFiducialId()).get();
            targetPoses.add(tag);
            Transform3d bestPose = target.getBestCameraToTarget();
            var camPose = tag.transformBy(bestPose.inverse());
            robotPoses.add(camPose.transformBy(new Transform3d(
                    new Translation3d(-Units.inchesToMeters(16), -Units.inchesToMeters(18.68),
                            Units.inchesToMeters(18.5)),
                    new Rotation3d(0, Math.toRadians(20), Math.toRadians(180)))));
        }
    }

    public List<Pose3d> getTagPoses() {
        return targetPoses;
    }

    public List<Pose3d> getRobotPoses() {
        return robotPoses;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("estimatedDistance", estimateDistance());
        SmartDashboard.putNumber("Distance Adjustment", distanceAssist());
        SmartDashboard.putNumber("Turning Adjustment", steeringAssist());
    }
}
