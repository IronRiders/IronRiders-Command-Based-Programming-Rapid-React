package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoCommandFactory {

  private final ShooterSubsystem shooter;
  private final DriveSubsystem drive;
  private final IntakeSubSystem intaker;
  private final IndexerSubsystem indexer;
  private final VisionSubsystem vision;

  public AutoCommandFactory(ShooterSubsystem shooter, DriveSubsystem drive, IntakeSubSystem intaker,
      IndexerSubsystem indexer, VisionSubsystem vision) {
    this.shooter = shooter;
    this.drive = drive;
    this.intaker = intaker;
    this.indexer = indexer;
    this.vision = vision;
  }

  public Command autoPath(String p, boolean initial) {
    PathPlannerTrajectory path = PathPlanner.loadPath(p, Constants.DRIVE_SPEED_AUTO, Constants.DRIVE_ACCELERATION_AUTO);
    Command cmd = new MecanumPathFollower(path, drive);
    if (initial) {
      cmd = cmd.beforeStarting(() -> drive.resetOdometry(new Pose2d(
          path.getInitialPose().getTranslation(),
          ((PathPlannerState) path.sample(0)).holonomicRotation)));
    }
    return cmd;
  }

  public SequentialCommandGroup fiveBallAuto() {

    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            autoPath("FiveBall1", true),
            //new RunCommand(intaker::intakeBall, intaker)),
      //  new ShooterTeleop(shooter, indexer, vision, drive),

       // new ParallelDeadlineGroup(
            autoPath("FiveBall2", false),
         //   new RunCommand(intaker::intakeBall, intaker)),

        autoPath("FiveBall3", false),
       // new ShooterTeleop(shooter, indexer, vision, drive),

      //  new ParallelDeadlineGroup(
            autoPath("FiveBall4", false),
        //    new RunCommand(intaker::intakeBall, intaker)),

        autoPath("FiveBall5", false)));
      //  new ShooterTeleop(shooter, indexer, vision, drive));
  }

  // Tuning/Debugging New Auto Paths

  public Command testAuto() {
    return autoPath("FiveBall1", true);
  }

  public Command TuningThetaController180() {
    return autoPath("TuningThetaControllerTest1(180)", true);
  }

  public Command TuningThetaController90() {
    return autoPath("TuningThetaControllerTest2(90)", true);
  }

  public Command TuningXYControllerTest1() {
    return autoPath("TuningXYControllerTest1", true);
  }
}
