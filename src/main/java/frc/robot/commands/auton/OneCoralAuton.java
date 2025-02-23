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
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.AutonConstants.AutonPositions;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.Robot;
import frc.robot.commands.auton.utils.AutonCommand;
import frc.robot.commands.auton.utils.AutonUtils;

public class OneCoralAuton extends AutonCommand {
    private AutonUtils utils;

    private final List<PathPlannerPath> paths;

    public OneCoralAuton(AutonUtils utils, AutonPositions position) {
        this.utils = utils;

        paths = getAutonPaths(position);

        if (Robot.isSimulation()) {
            addCommands(utils.resetOdometry(paths.get(0)));
        }

        addCommands(
            Commands.sequence(
                Commands.parallel(
                    utils.goToElevatorState(ElevatorStates.TOP),
                    utils.goToArmState(ArmStates.TOP)
                ),
                AutoBuilder.followPath(paths.get(0)),
                utils.scoreCoralCommand(),
                Commands.waitSeconds(1.0),
                AutoBuilder.followPath(paths.get(1)),
                Commands.parallel(
                    utils.goToElevatorState(ElevatorStates.PICKUP),
                    utils.goToArmState(ArmStates.BOTTOM)
                )
            )
        );
    }

    private List<PathPlannerPath> getAutonPaths(AutonPositions position) {
        return switch (position) {
            case TOP -> List.of(utils.loadPath("TopOneCoral-1"), utils.loadPath("TopOneCoral-2"));
            case CENTER -> List.of(utils.loadPath("CenterOneCoral-1"), utils.loadPath("CenterOneCoral-2"));
            case BOTTOM -> List.of(utils.loadPath("BottomOneCoral-1"), utils.loadPath("BottomOneCoral-2"));
            default -> null;
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
