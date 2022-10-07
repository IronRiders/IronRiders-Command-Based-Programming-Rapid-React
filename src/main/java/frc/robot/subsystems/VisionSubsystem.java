package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
    private NetworkTable table;
    private PIDController pidController;

    public VisionSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        pidController = new PIDController(Constants.TURN_P, 0.0, 0.0);
        pidController.setSetpoint(0);
        pidController.setTolerance(Constants.TURN_TOLERANCE);
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
        SmartDashboard.putBoolean("in shooter range", (distance < 165 && distance > 85));
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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("estimatedDistance", estimateDistance());
        SmartDashboard.putNumber("Distance Adjustment", distanceAssist());
        SmartDashboard.putNumber("Turning Adjustment", steeringAssist());
    }
}
