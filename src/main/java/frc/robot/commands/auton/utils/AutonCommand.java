package frc.robot.commands.auton.utils;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotStates.StartingPosition;

public abstract class AutonCommand extends SequentialCommandGroup {
    public abstract List<Pose2d> getAllPathPoses();
    public abstract Pose2d getStartingPose();

    /**
     * Returns the correct auton paths based on the starting position of the robot.
     * @param position The starting position of the robot.
     * @return The auton path that the robot should drive.
     */
    protected abstract List<PathPlannerPath> getAutonPaths(StartingPosition position);
}
