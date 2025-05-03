package frc.robot.commands.drivebase;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutonConstants.DashboardAlignment;
import frc.robot.commands.drivebase.base.DriveToPose;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.utils.PoseNavigator;

public class DriveToDashboardPose extends DeferredCommand {

    public DriveToDashboardPose(SwerveSubsystem drivebase, PoseNavigator poseNavigator) {
        super(() ->
            new SequentialCommandGroup(
                new DriveToPose(
                    drivebase,
                    poseNavigator.calculateReefPose(
                        poseNavigator.getTargetDashboardPose(),
                        DashboardAlignment.DISTANCE_AWAY_REEF
                    )
                ),
                new DriveToPose(
                    drivebase,
                    poseNavigator.calculateReefPose(
                        poseNavigator.getTargetDashboardPose(),
                        DashboardAlignment.DISTANCE_AT_REEF
                    ), 2.25, 2.5
                )
            ), Set.of(drivebase, poseNavigator)
        );
    }
}
