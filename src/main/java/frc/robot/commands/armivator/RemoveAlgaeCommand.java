package frc.robot.commands.armivator;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.RobotStates.ArmStates;
import frc.robot.Constants.RobotStates.ElevatorStates;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.PoseNavigator;

public class RemoveAlgaeCommand extends SequentialCommandGroup {
    
    public RemoveAlgaeCommand(ArmivatorCommands armivatorCommands, SwerveSubsystem drivebase, PoseNavigator poseNavigator) {
        addCommands(
            Commands.runOnce(() -> drivebase.lock()),
            armivatorCommands.setArmivatorState(ElevatorStates.BOTTOM, ArmStates.L2),
            Commands.parallel(
                Commands.either(
                    armivatorCommands.adjustArmSetpoint(-50).withTimeout(0.25),
                    armivatorCommands.adjustArmSetpoint(-210).withTimeout(0.5),
                    poseNavigator::isHighAlgae
                ),
                Commands.defer(
                    () -> drivebase.driveToPose(poseNavigator.calculateClosestReefPose(), AutonConstants.MAX_VELOCITY, AutonConstants.MAX_ACCELERATION),
                    Set.of(drivebase)
                )
            ),
            armivatorCommands.adjustArmSetpoint(230).withTimeout(0.5),
            drivebase.driveToDistance(-0.75, 2.5, 2.0)
        );
    }
}
