package frc.robot.commands.auton;

import java.util.Collections;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.RobotStates.StartingPosition;
import frc.robot.commands.auton.utils.AutonCommand;

public class NoneAuton extends AutonCommand {
    
    public NoneAuton() {
        addCommands(Commands.none());
    }

    @Override
    public List<Pose2d> getAllPathPoses() {
        return Collections.emptyList();
    }

    @Override
    public Pose2d getStartingPose() {
        return new Pose2d();
    }

    @Override
    protected List<PathPlannerPath> getAutonPaths(StartingPosition position) {
        return Collections.emptyList();
    }
}
