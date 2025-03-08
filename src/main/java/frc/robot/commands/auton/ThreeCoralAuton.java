package frc.robot.commands.auton;

import java.util.Collection;
import java.util.Collections;
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

public class ThreeCoralAuton extends AutonCommand {
    private AutonUtils utils;

    private final List<PathPlannerPath> paths;

    public ThreeCoralAuton(AutonUtils utils, StartingPosition position) {
        this.utils = utils;

        paths = getAutonPaths(position);

        if (Robot.isSimulation()) {
            addCommands(utils.resetOdometry(paths.get(0)));
        }

        addCommands(
            Commands.sequence(
                Commands.parallel(
                    utils.setArmivatorState(ElevatorStates.TOP, ArmStates.TOP),
                    AutoBuilder.followPath(paths.get(0))
                ),
                utils.scoreCoralCommand(0.3),
                Commands.parallel(
                    AutoBuilder.followPath(paths.get(1)),
                    Commands.sequence(
                        Commands.waitSeconds(0.75),
                        utils.setArmivatorState(ElevatorStates.PICKUP, ArmStates.BOTTOM)
                    )
                ),
                utils.runSerializerCommand(1.5), // TODO: Implement serializer sensor.
                utils.setElevatorState(ElevatorStates.BOTTOM, 0.8),
                Commands.parallel(
                    utils.setArmivatorState(ElevatorStates.TOP, ArmStates.TOP),
                    AutoBuilder.followPath(paths.get(2))
                ),
                utils.scoreCoralCommand(0.3),
                Commands.parallel(
                    AutoBuilder.followPath(paths.get(3)),
                    Commands.sequence(
                        Commands.waitSeconds(0.75),
                        utils.setArmivatorState(ElevatorStates.BOTTOM, ArmStates.BOTTOM)
                    )
                )
            )
        );
    }

    @Override
    protected List<PathPlannerPath> getAutonPaths(StartingPosition position) {
        return switch (position) {
            case LEFT -> List.of(
                utils.loadPath("SideThreeCoral-1").mirrorPath(),
                utils.loadPath("SideThreeCoral-2").mirrorPath(),
                utils.loadPath("SideThreeCoral-3").mirrorPath(),
                utils.loadPath("SideThreeCoral-4").mirrorPath(),
                utils.loadPath("SideThreeCoral-5").mirrorPath()
            );
            case CENTER -> Collections.emptyList();
            case RIGHT -> List.of(
                utils.loadPath("SideThreeCoral-1"),
                utils.loadPath("SideThreeCoral-2"),
                utils.loadPath("SideThreeCoral-3"),
                utils.loadPath("SideThreeCoral-4"),
                utils.loadPath("SideThreeCoral-5")
            );
        };
    } 

    @Override
    public List<Pose2d> getAllPathPoses() {
        return paths.subList(0, 4).stream()
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
