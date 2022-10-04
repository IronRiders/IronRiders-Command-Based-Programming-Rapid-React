package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    private boolean inverted;
    private MecanumWheel[] motors = new MecanumWheel[4];
    private final MecanumDriveKinematics kinematics;
    private final MecanumDriveOdometry odometry;
    private final AHRS navx;

    private static ProfiledPIDController thetaController = new ProfiledPIDController(Constants.AUTO_THETACONTROLLER_KP,
            0, 0,
            new TrapezoidProfile.Constraints(Units.rotationsToRadians(0.75), Units.rotationsToRadians(1.5)));
    private static PIDController xController = new PIDController(Constants.AUTO_POSITION_KP, 0, 0);
    private static PIDController yController = new PIDController(Constants.AUTO_POSITION_KP, 0, 0);

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

        navx = new AHRS();
        odometry = new MecanumDriveOdometry(kinematics, new Rotation2d());
        thetaController.enableContinuousInput(-180, 180); // For more efficiency when turning.
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
        // Tuning
        NetworkTableInstance.getDefault().flush();
        odometry.update(navx.getRotation2d(), getWheelSpeeds());
        SmartDashboard.putNumber("x controller", getPose2d().getX());
        SmartDashboard.putNumber("x Controller (target)", yController.getSetpoint());
        SmartDashboard.putNumber("Y controller", getPose2d().getY());
        SmartDashboard.putNumber("y Controller (target)", xController.getSetpoint());
        SmartDashboard.putNumber("Theta controller (Degrees)", getPose2d().getRotation().getDegrees());
        SmartDashboard.putNumber("Theta setPoint (Target))", Math.toDegrees(thetaController.getSetpoint().position));
    }

    public Pose2d getPose2d() {
        return odometry.getPoseMeters();
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
        odometry.resetPosition(pose2d, navx.getRotation2d());
    }

    public ProfiledPIDController getThetaController() {
        return thetaController;
    }

    public PIDController getxController() {
        return xController;
    }

    public PIDController getyController() {
        return yController;
    }
}
