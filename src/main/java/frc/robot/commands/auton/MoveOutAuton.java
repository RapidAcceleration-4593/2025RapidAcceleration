package frc.robot.commands.auton;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.Constants.RobotStates.StartingPosition;
import frc.robot.commands.auton.utils.AutonCommand;
import frc.robot.commands.auton.utils.AutonUtils;

public class MoveOutAuton extends AutonCommand {
    private AutonUtils utils;

    private final List<PathPlannerPath> paths;

    public MoveOutAuton(AutonUtils utils, StartingPosition position) {
        this.utils = utils;

        paths = getAutonPaths(position);

        if (Robot.isSimulation()) {
            addCommands(utils.resetOdometry(paths.get(0)));
        }

        addCommands(
            Commands.sequence(
                AutoBuilder.followPath(paths.get(0))
            )
        );
    }

    @Override
    protected List<PathPlannerPath> getAutonPaths(StartingPosition position) {
        return switch (position) {
            case LEFT -> List.of(utils.loadPath("SideMoveOut-1").mirrorPath());
            case CENTER -> List.of(utils.loadPath("CenterMoveOut-1"));
            case RIGHT -> List.of(utils.loadPath("SideMoveOut-1"));
        };
    } 

    @Override
    public List<Pose2d> getAllPathPoses() {
        return paths.subList(0, 0).stream()
            .map(PathPlannerPath::getPathPoses)
            .flatMap(Collection::stream)
            .collect(Collectors.toList());
    }

    @Override
    public Pose2d getStartingPose() {
        return paths.get(0)
            .generateTrajectory(new ChassisSpeeds(), new Rotation2d(), utils.getRobotConfig())
            .getInitialPose();
    }
}
