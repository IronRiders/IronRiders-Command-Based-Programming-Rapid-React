package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MecanumWheel;

public class MecanumPathFollower extends PPMecanumControllerCommand {

    public MecanumPathFollower(PathPlannerTrajectory trajectory, DriveSubsystem drive) {
        super(trajectory,
                () -> drive.getPose2d(),
                drive.getKinematics(),
                drive.getxController(), drive.getyController(), drive.getThetaController(),
                MecanumWheel.getMaxLinearVelocity(), (speed) -> drive.SetWheelSpeeds(speed), drive);
    }
}
