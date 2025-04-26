package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotStates.ArmStates;
import frc.robot.Constants.RobotStates.ElevatorStates;
import frc.robot.commands.drivebase.DriveToClosestReef;
import frc.robot.commands.drivebase.DriveToDistance;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.utils.PoseNavigator;

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
                new DriveToClosestReef(drivebase, poseNavigator)
            ),
            armivatorCommands.adjustArmSetpoint(230).withTimeout(0.5),
            new DriveToDistance(drivebase, -0.75).withTimeout(0.8)
        );
    }
}
