package frc.robot.commands.armivator;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutonConstants.DashboardAlignment;
import frc.robot.Constants.RobotStates.ArmStates;
import frc.robot.Constants.RobotStates.ElevatorStates;
import frc.robot.commands.armivator.base.ArmivatorCommands;
import frc.robot.commands.drivebase.DriveToDistance;
import frc.robot.commands.drivebase.base.DriveToPose;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.utils.PoseNavigator;

public class RemoveAlgaeCommand extends DeferredCommand {
    
    public RemoveAlgaeCommand(SwerveSubsystem drivebase, ArmivatorCommands armivatorCommands, PoseNavigator poseNavigator) {
        super(() ->
            new SequentialCommandGroup(
                Commands.parallel(
                    new DriveToPose(drivebase, poseNavigator.calculateClosestReefPose(DashboardAlignment.DISTANCE_AWAY_REEF)),
                    Commands.either(
                        armivatorCommands.setArmivatorState(ElevatorStates.BOTTOM, ArmStates.HIGH_ALGAE),
                        armivatorCommands.setArmivatorState(ElevatorStates.BOTTOM, ArmStates.LOW_ALGAE),
                        poseNavigator::isHighAlgae
                    )
                ),
                new DriveToPose(drivebase, poseNavigator.calculateClosestReefPose(DashboardAlignment.DISTANCE_AT_REEF)),
                Commands.parallel(
                    Commands.either(
                        armivatorCommands.setArmivatorState(ElevatorStates.BOTTOM, ArmStates.TOP),
                        armivatorCommands.setArmivatorState(ElevatorStates.BOTTOM, ArmStates.HIGH_ALGAE),
                        poseNavigator::isHighAlgae
                    ),
                    new DriveToDistance(drivebase, -1.25).withTimeout(1.25)
                )
            ), Set.of(drivebase, poseNavigator)
        );
    }
}
