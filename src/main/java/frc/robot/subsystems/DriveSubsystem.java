package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    private boolean inverted;
    private ChassisSpeeds ActualChassisSpeeds;
    private ChassisSpeeds targetChassisSpeeds;
    private MecanumWheel[] motors = new MecanumWheel[4];
    private final MecanumDriveKinematics kinematics;
    private final WPI_Pigeon2 pigeon;
    public Field2d field;
    private static ProfiledPIDController profiledThetaController = new ProfiledPIDController(Constants.AUTO_THETACONTROLLER_KP,
            0, 0,
            new TrapezoidProfile.Constraints(Units.rotationsToRadians(0.75), Units.rotationsToRadians(1.5)));
    private static PIDController thetaController = new PIDController(profiledThetaController.getP(), profiledThetaController.getI(), profiledThetaController.getD());       
    private static PIDController xController = new PIDController(Constants.AUTO_POSITION_KP, 0, 0);
    private static PIDController yController = new PIDController(Constants.AUTO_POSITION_KP, 0, 0);

    private final MecanumDrivePoseEstimator poseEstimator;
    private static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0, 0, Units.degreesToRadians(0));
    private static final Matrix<N1, N1> localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0));
    private static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0, 0, Units.degreesToRadians(0));
    public final PhotonCamera camera = new PhotonCamera("Camera Name");
    public PhotonPipelineResult previousPipelineResult = null;

    public static final List<Pose3d> allTargetPoses = List.of(
            new Pose3d(new Translation3d(-0.0035306, 7.578928199999999, .8858503999999999), (new Rotation3d(0, 0, 0))));

    public DriveSubsystem() {
        motors[0] = new MecanumWheel(Constants.WHEEL_PORT_FRONT_LEFT, true);
        motors[1] = new MecanumWheel(Constants.WHEEL_PORT_FRONT_RIGHT, false);
        motors[2] = new MecanumWheel(Constants.WHEEL_PORT_REAR_LEFT, true);
        motors[3] = new MecanumWheel(Constants.WHEEL_PORT_REAR_RIGHT, false);
        inverted = false;

        // meter per second
        kinematics = new MecanumDriveKinematics(
                new Translation2d(0.28575, 0.2267),
                new Translation2d(0.28575, -0.2267),
                new Translation2d(-0.28575, 0.2267),
                new Translation2d(-0.28575, -0.2267));

        pigeon = new WPI_Pigeon2(15);
        targetChassisSpeeds = new ChassisSpeeds();
        profiledThetaController.enableContinuousInput(-Math.PI, Math.PI); // For more efficiency when turning.
        field = new Field2d();

        poseEstimator = new MecanumDrivePoseEstimator(
                pigeon.getRotation2d(),
                new Pose2d(),
                getKinematics(), stateStdDevs,
                localMeasurementStdDevs, visionMeasurementStdDevs);
        field = new Field2d();
    }

    public void invertDrive() {
        inverted = !inverted;
    }

    public MecanumDriveWheelSpeeds getWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(motors[0].getVelocity(),
                motors[1].getVelocity(),
                motors[2].getVelocity(),
                motors[3].getVelocity());
    }

    @Override
    public void periodic() {
            // Update pose estimator with drivetrain sensors
            poseEstimator.updateWithTime(
                    Timer.getFPGATimestamp(),
                    pigeon.getRotation2d(),
                    getWheelSpeeds());

            field.setRobotPose(getPose2d());

        // Tuning
        NetworkTableInstance.getDefault().flush();
        poseEstimator.update(pigeon.getRotation2d(), getWheelSpeeds());
        SmartDashboard.putNumber("x controller", getPose2d().getX());
        SmartDashboard.putNumber("x Controller (target)", xController.getSetpoint());
        SmartDashboard.putNumber("Y controller", getPose2d().getY());
        SmartDashboard.putNumber("y Controller (target)", yController.getSetpoint());
        SmartDashboard.putNumber("Theta controller (Degrees)", getPose2d().getRotation().getDegrees());
        SmartDashboard.putNumber("Theta setPoint (Target))", Math.toDegrees(profiledThetaController.getSetpoint().position));

        ActualChassisSpeeds = kinematics.toChassisSpeeds(getWheelSpeeds());

        SmartDashboard.putNumber("Drive/VX", ActualChassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Drive/VY", ActualChassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Drive/Omega Degrees",
                Units.radiansToDegrees(ActualChassisSpeeds.omegaRadiansPerSecond));

        SmartDashboard.putNumber("Drive/Target VX", targetChassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Drive/Target VY", targetChassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Drive/Target Omega Degrees",
                Units.radiansToDegrees(targetChassisSpeeds.omegaRadiansPerSecond));

        field.getObject("Target").setPose(
                new Pose2d(
                        xController.getSetpoint(),
                        yController.getSetpoint(),
                        new Rotation2d(profiledThetaController.getSetpoint().position)));
    }

    public Pose2d getPose2d() {
        return poseEstimator.getEstimatedPosition();
    }

    public MecanumDriveKinematics getKinematics() {
        return kinematics;
    }

    public void SetWheelSpeeds(MecanumDriveWheelSpeeds speed) {
        motors[0].setVelocity(speed.frontLeftMetersPerSecond);
        motors[1].setVelocity(speed.frontRightMetersPerSecond);
        motors[2].setVelocity(speed.rearLeftMetersPerSecond);
        motors[3].setVelocity(speed.rearRightMetersPerSecond);
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        targetChassisSpeeds = chassisSpeeds;
        SetWheelSpeeds(kinematics.toWheelSpeeds(chassisSpeeds));
    }

    public void setChassisSpeeds(double strafe, double drive, double turn, boolean useInverted) {
        if (useInverted && inverted) {
            drive = -drive;
            strafe = -strafe;
        }

        double xspeed = drive * MecanumWheel.getMaxLinearVelocity();
        double yspeed = strafe * MecanumWheel.getMaxLinearVelocity();
        double turnspeed = turn * -getMaxRotationalVelocity();

        // Debugging
        SmartDashboard.putNumber("DriveSubsystem/xSpeed", xspeed);
        SmartDashboard.putNumber("DriveSubsystem/ySpeed", yspeed);
        SmartDashboard.putNumber("DriveSubsystem/turnSpeed", turnspeed);

        setChassisSpeeds(new ChassisSpeeds(xspeed, yspeed, turnspeed));
    }

    public double getMaxRotationalVelocity() {
        return Math.abs((kinematics.toChassisSpeeds(new MecanumDriveWheelSpeeds(
                MecanumWheel.getMaxLinearVelocity(),
                -MecanumWheel.getMaxLinearVelocity(),
                MecanumWheel.getMaxLinearVelocity(),
                -MecanumWheel.getMaxLinearVelocity()))).omegaRadiansPerSecond);
    }

    public void stop() {
        setChassisSpeeds(0, 0, 0, true);
    }

    public void resetOdometry(Pose2d pose2d) {
        poseEstimator.resetPosition(pose2d, pigeon.getRotation2d());
    }

    public ProfiledPIDController getProfiledThetaController() {
        return profiledThetaController;
    }

    public PIDController getThetaController() {
        return thetaController;
    }

    public PIDController getxController() {
        return xController;
    }

    public PIDController getyController() {
        return yController;
    }

    public MecanumDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }
}
