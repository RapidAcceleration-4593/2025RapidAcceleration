package frc.robot.commands.auton;

import java.util.Collection;
import java.util.List;
import java.util.Map;
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
import frc.robot.commands.armivator.ArmivatorCommands;
import frc.robot.commands.auton.utils.AutonCommand;
import frc.robot.commands.auton.utils.AutonUtils;

public class OneCoralAuton extends AutonCommand {

    private final AutonUtils utils;
    private final List<PathPlannerPath> paths;

    public OneCoralAuton(ArmivatorCommands armivatorCommands, AutonUtils utils, StartingPosition position) {
        this.utils = utils;
        this.paths = getAutonPaths(position);

        if (Robot.isSimulation()) {
            addCommands(utils.resetOdometry(paths.get(0)));
        }

        addCommands(
            Commands.parallel(
                armivatorCommands.setArmivatorState(ElevatorStates.TOP, ArmStates.TOP),
                Commands.sequence(
                    Commands.waitSeconds(2.0),
                    AutoBuilder.followPath(paths.get(0))
                )
            ),
            armivatorCommands.scoreCoral(),
            Commands.parallel(
                utils.driveBackward(),
                Commands.sequence(
                    Commands.waitSeconds(0.75),
                    armivatorCommands.setArmivatorState(ElevatorStates.BOTTOM, ArmStates.BOTTOM)
                )
            )
        );
    }

    @Override
    protected List<PathPlannerPath> getAutonPaths(StartingPosition position) {
        return Map.of(
            StartingPosition.LEFT, List.of(utils.loadPath("SideCoral-1")),
            StartingPosition.CENTER, List.of(utils.loadPath("CenterCoral-1")),
            StartingPosition.RIGHT, List.of(utils.loadPath("SideCoral-1").mirrorPath())
        ).getOrDefault(position, List.of());
    } 

    @Override
    public List<Pose2d> getAllPathPoses() {
        return paths.stream()
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
