package frc.robot;

import java.util.stream.Collectors;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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
        public final ChaseAprilTag chase = new ChaseAprilTag(drive);

        AutoCommandFactory autocmdFactory = new AutoCommandFactory(shooter, drive, intake, indexer, vision);

        public RobotContainer() {

                SmartDashboard.putNumber("Shooter Offset RPM", 0);

                drive.setDefaultCommand(
                                new RunCommand(() -> drive.setChassisSpeeds(joystickResponse(controller.getRawAxis(0)),
                                                joystickResponse(controller.getRawAxis(1)),
                                                joystickResponse(controller.getRawAxis(3)), true), drive));
               // shooter.setDefaultCommand(new RunCommand(() -> shooter.shoot(50), shooter));

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
                return autocmdFactory.fiveBallAuto();
        }

        public void traj() {
                SmartDashboard.putData("field", drive.field);
                drive.field.getObject("bestPoses").setPoses(
                                vision.getRobotPoses().stream().map(p -> p.toPose2d()).collect(Collectors.toList()));
                drive.field.getObject("targetPoses").setPoses(
                                vision.getTagPoses().stream().map(p -> p.toPose2d()).collect(Collectors.toList()));
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

        public Pose3d toPose3d(Pose2d pose) {
                return new Pose3d(new Translation3d(pose.getX(), pose.getY(), 0.0),
                                new Rotation3d(0.0, 0.0, pose.getRotation().getRadians()));
        }

    public void periodic() {
           // Update pose estimator with visible targets
           var pipelineResult = vision.camera.getLatestResult();
           if (!pipelineResult.equals(vision.previousPipelineResult) && pipelineResult.hasTargets()) {
               vision.previousPipelineResult = pipelineResult;
           //    double imageCaptureTime = Timer.getFPGATimestamp() - (pipelineResult.getLatencyMillis() / 1000d);
   
               for (PhotonTrackedTarget target : pipelineResult.getTargets()) {
   
                   var fiducialId = target.getFiducialId();
                   if (fiducialId >= 0 && fiducialId < vision.allTargetPoses.size()) { 
                   //    var targetPose = vision.allTargetPoses.get(fiducialId);
   
                       Transform3d camToTarget = target.getBestCameraToTarget();
                   //    Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

                       var robotToCamera = (new Transform3d(
                        new Translation3d(-Units.inchesToMeters(16), -Units.inchesToMeters(18.68),
                                Units.inchesToMeters(18.5)),
                        new Rotation3d(0, Math.toRadians(20), Math.toRadians(180))));
   
                     //  Pose3d visionMeasurement = camPose.transformBy(robotToCamera.inverse());
                        chase.Update(toPose3d(drive.getPose2d()).transformBy(robotToCamera).transformBy(camToTarget));
                      // drive.getPoseEstimator().addVisionMeasurement(visionMeasurement.toPose2d(), imageCaptureTime);
                   }
               }
           }

        }
}
