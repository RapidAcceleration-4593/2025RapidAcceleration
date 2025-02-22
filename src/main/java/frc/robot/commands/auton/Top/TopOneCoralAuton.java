package frc.robot.commands.auton.Top;

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
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.commands.auton.utils.AutonCommand;
import frc.robot.commands.auton.utils.AutonUtils;

public class TopOneCoralAuton extends AutonCommand {
    private AutonUtils utils;

    private final List<PathPlannerPath> paths;

    public TopOneCoralAuton(AutonUtils utils) {
        this.utils = utils;

        paths = List.of(
            utils.loadPath("TopOneCoral-1"),
            utils.loadPath("TopOneCoral-2")
        );

        if (Robot.isSimulation()) {
            addCommands(utils.resetOdometry(paths.get(0)));
        }

        addCommands(
            Commands.sequence(
                utils.setElevatorState(ElevatorStates.PICKUP),
                Commands.parallel(
                    AutoBuilder.followPath(paths.get(0)),
                    utils.setArmState(ArmStates.TOP)
                ),
                utils.setElevatorState(ElevatorStates.BOTTOM),
                utils.scoreCoralCommand(),
                Commands.waitSeconds(1.0),
                Commands.parallel(
                    AutoBuilder.followPath(paths.get(1)),
                    utils.setArmState(ArmStates.BOTTOM),
                    utils.setElevatorState(ElevatorStates.BOTTOM)
                )
            )
        );
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
