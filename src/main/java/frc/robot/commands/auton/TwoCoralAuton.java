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
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.AutonConstants.AutonPositions;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.Robot;
import frc.robot.commands.auton.utils.AutonCommand;
import frc.robot.commands.auton.utils.AutonUtils;

public class TwoCoralAuton extends AutonCommand {
    private AutonUtils utils;

    private final List<PathPlannerPath> paths;

    public TwoCoralAuton(AutonUtils utils, AutonPositions position) {
        this.utils = utils;

        paths = getAutonPaths(position);

        if (Robot.isSimulation()) {
            addCommands(utils.resetOdometry(paths.get(0)));
        }

        addCommands(
            Commands.sequence(
                utils.goToElevatorState(ElevatorStates.PICKUP),
                Commands.parallel(
                    utils.goToElevatorState(ElevatorStates.TOP),
                    utils.goToArmState(ArmStates.TOP),
                    Commands.sequence(
                        Commands.waitSeconds(0.25),
                        AutoBuilder.followPath(paths.get(0))
                    )
                ),
                utils.scoreCoralCommand(),
                Commands.parallel(
                    AutoBuilder.followPath(paths.get(1)),
                    utils.goToElevatorState(ElevatorStates.PICKUP),
                    utils.goToArmState(ArmStates.BOTTOM)
                ),
                utils.runSerializerCommand(1.5),
                utils.goToElevatorState(ElevatorStates.BOTTOM),
                utils.goToElevatorState(ElevatorStates.PICKUP),
                Commands.parallel(
                    utils.goToElevatorState(ElevatorStates.TOP),
                    utils.goToArmState(ArmStates.TOP),
                    Commands.sequence(
                        Commands.waitSeconds(0.25),
                        AutoBuilder.followPath(paths.get(2))
                    )
                ),
                utils.scoreCoralCommand(),
                Commands.parallel(
                    AutoBuilder.followPath(paths.get(3)),
                    utils.goToElevatorState(ElevatorStates.PICKUP),
                    utils.goToArmState(ArmStates.BOTTOM)
                ),
                utils.goToElevatorState(ElevatorStates.BOTTOM)
            )
        );
    }

    private List<PathPlannerPath> getAutonPaths(AutonPositions position) {
        return switch (position) {
            case LEFT -> List.of(utils.loadPath("LeftTwoCoral-1"), utils.loadPath("LeftTwoCoral-2"), utils.loadPath("LeftTwoCoral-3"), utils.loadPath("LeftTwoCoral-4"));
            case CENTER -> Collections.emptyList();
            case RIGHT -> List.of(utils.loadPath("RightTwoCoral-1"), utils.loadPath("RightTwoCoral-2"), utils.loadPath("RightTwoCoral-3"), utils.loadPath("RightTwoCoral-4"));
        };
    } 

    @Override
    public List<Pose2d> getAllPathPoses() {
        return paths.subList(0, 3).stream()
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
