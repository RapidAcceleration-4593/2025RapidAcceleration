package frc.robot.commands.auton.utils;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotStates.StartingPosition;

public abstract class AutonCommand extends SequentialCommandGroup {

    public abstract List<Pose2d> getAllPathPoses();
    public abstract Pose2d getStartingPose();
    protected abstract List<PathPlannerPath> getAutonPaths(StartingPosition position);
}
