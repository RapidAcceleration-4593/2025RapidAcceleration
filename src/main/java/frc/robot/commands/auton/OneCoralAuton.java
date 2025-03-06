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
import frc.robot.Constants.RobotStates.Arm.ArmStates;
import frc.robot.Constants.RobotStates.Autonomous.StartingPosition;
import frc.robot.Constants.RobotStates.Elevator.ElevatorStates;
import frc.robot.Robot;
import frc.robot.commands.auton.utils.AutonCommand;
import frc.robot.commands.auton.utils.AutonUtils;

public class OneCoralAuton extends AutonCommand {
    private AutonUtils utils;

    private final List<PathPlannerPath> paths;

    public OneCoralAuton(AutonUtils utils, StartingPosition position) {
        this.utils = utils;

        paths = getAutonPaths(position);

        if (Robot.isSimulation()) {
            addCommands(utils.resetOdometry(paths.get(0)));
        }

        addCommands(
            Commands.sequence(
                Commands.parallel(
                    utils.goToArmivatorState(ArmStates.TOP, ElevatorStates.TOP),
                    AutoBuilder.followPath(paths.get(0))
                ),
                utils.scoreCoralCommand(),
                Commands.parallel(
                    AutoBuilder.followPath(paths.get(1)),
                    utils.goToArmivatorState(ArmStates.BOTTOM, ElevatorStates.PICKUP)
                )
            )
        );
    }

    @Override
    protected List<PathPlannerPath> getAutonPaths(StartingPosition position) {
        return switch (position) {
            case LEFT -> List.of(
                utils.loadPath("SideOneCoral-1").mirrorPath(),
                utils.loadPath("SideOneCoral-2").mirrorPath()
            );
            case CENTER -> List.of(
                utils.loadPath("CenterOneCoral-1"),
                utils.loadPath("CenterOneCoral-2")
            );
            case RIGHT -> List.of(
                utils.loadPath("SideOneCoral-1"),
                utils.loadPath("SideOneCoral-2")
            );
        };
    } 

    @Override
    public List<Pose2d> getAllPathPoses() {
        return paths.subList(0, 1).stream()
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
