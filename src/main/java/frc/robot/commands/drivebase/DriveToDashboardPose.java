package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutonConstants.DashboardAlignment;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.utils.PoseNavigator;

public class DriveToDashboardPose extends SequentialCommandGroup {

    public DriveToDashboardPose(SwerveSubsystem drivebase, PoseNavigator poseNavigator) {
        addCommands(
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
        );
    }
    
}
