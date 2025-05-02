package frc.robot.commands.auton;

import java.util.Collection;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SerializerConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorTravelTime;
import frc.robot.Constants.RobotStates.ArmStates;
import frc.robot.Constants.RobotStates.StartingPosition;
import frc.robot.Constants.RobotStates.ElevatorStates;
import frc.robot.commands.armivator.ArmivatorCommands;
import frc.robot.commands.auton.utils.AutonCommand;
import frc.robot.commands.auton.utils.AutonUtils;

public class TwoHalfCoralAuton extends AutonCommand {

    private final AutonUtils utils;
    private final List<PathPlannerPath> paths;

    public TwoHalfCoralAuton(ArmivatorCommands armivatorCommands, AutonUtils utils, StartingPosition position) {
        this.utils = utils;
        this.paths = getAutonPaths(position);

        if (RobotBase.isSimulation()) {
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
                    Commands.waitSeconds(0.8),
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
                    Commands.waitSeconds(0.8),
                    armivatorCommands.setArmivatorState(ElevatorStates.BOTTOM, ArmStates.BOTTOM)
                )
            )
        );
    }

    @Override
    protected List<PathPlannerPath> getAutonPaths(StartingPosition position) {
        return switch (position) {
            case LEFT -> List.of(
                utils.loadPath("LeftCoral-1"),
                utils.loadPath("LeftCoral-2"),
                utils.loadPath("LeftCoral-3"),
                utils.loadPath("LeftCoral-4")
            );
            case CENTER -> List.of();
            case RIGHT -> List.of(
                utils.loadPath("RightCoral-1"),
                utils.loadPath("RightCoral-2"),
                utils.loadPath("RightCoral-3"),
                utils.loadPath("RightCoral-4")
            );
        };
    } 

    @Override
    public List<Pose2d> getAllPathPoses() {
        return paths.stream()
            .map(PathPlannerPath::getPathPoses)
            .flatMap(Collection::stream)
            .toList();
    }

    @Override
    public Pose2d getStartingPose() {
        return paths.get(0)
            .generateTrajectory(new ChassisSpeeds(), new Rotation2d(), utils.getRobotConfig())
            .getInitialPose();
    }
}
