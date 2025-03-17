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
import frc.robot.Constants.SerializerConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorTravelTime;
import frc.robot.Constants.RobotStates.ArmStates;
import frc.robot.Constants.RobotStates.StartingPosition;
import frc.robot.Constants.RobotStates.ElevatorStates;
import frc.robot.Robot;
import frc.robot.commands.armivator.ArmivatorCommands;
import frc.robot.commands.auton.utils.AutonCommand;
import frc.robot.commands.auton.utils.AutonUtils;

public class TwoHalfCoralAuton extends AutonCommand {

    private final AutonUtils utils;
    private final List<PathPlannerPath> paths;

    public TwoHalfCoralAuton(ArmivatorCommands armivatorCommands, AutonUtils utils, StartingPosition position) {
        this.utils = utils;
        this.paths = getAutonPaths(position);

        if (Robot.isSimulation()) {
            addCommands(utils.resetOdometry(paths.get(0)));
        }

        addCommands(
            Commands.parallel(
                armivatorCommands.setArmivatorState(ElevatorStates.TOP, ArmStates.TOP),
                AutoBuilder.followPath(paths.get(0))
            ),
            armivatorCommands.scoreCoral(),
            Commands.parallel(
                AutoBuilder.followPath(paths.get(1)),
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    armivatorCommands.setArmivatorState(ElevatorStates.PICKUP, ArmStates.BOTTOM)
                )
            ),
            armivatorCommands.runSerializerCommand().withTimeout(SerializerConstants.MAX_TIMEOUT),
            armivatorCommands.setElevatorState(ElevatorStates.BOTTOM).withTimeout(ElevatorTravelTime.KAH_CHUNK),
            Commands.parallel(
                AutoBuilder.followPath(paths.get(2)),
                armivatorCommands.setArmivatorState(ElevatorStates.TOP, ArmStates.TOP)
            ),
            armivatorCommands.scoreCoral(),
            Commands.parallel(
                AutoBuilder.followPath(paths.get(3)),
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    armivatorCommands.setArmivatorState(ElevatorStates.PICKUP, ArmStates.BOTTOM)
                )
            ),
            armivatorCommands.runSerializerCommand().withTimeout(SerializerConstants.MAX_TIMEOUT),
            armivatorCommands.setElevatorState(ElevatorStates.BOTTOM).withTimeout(ElevatorTravelTime.KAH_CHUNK)
        );
    }

    @Override
    protected List<PathPlannerPath> getAutonPaths(StartingPosition position) {
        return Map.of(
            StartingPosition.LEFT, List.of(
                utils.loadPath("LeftSideCoral-1"),
                utils.loadPath("LeftSideCoral-2"),
                utils.loadPath("LeftSideCoral-3"),
                utils.loadPath("LeftSideCoral-4"),
                utils.loadPath("LeftSideCoral-5")
            ),
            StartingPosition.CENTER, List.<PathPlannerPath>of(),
            StartingPosition.RIGHT, List.of(
                utils.loadPath("RightSideCoral-1"),
                utils.loadPath("RightSideCoral-2"),
                utils.loadPath("RightSideCoral-3"),
                utils.loadPath("RightSideCoral-4"),
                utils.loadPath("RightSideCoral-5")
            )
        ).getOrDefault(position, List.<PathPlannerPath>of());
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
