package frc.robot.commands.armivator;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotStates.Arm.ArmStates;
import frc.robot.Constants.RobotStates.Elevator.ElevatorStates;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.PoseNavigator;

public class RemoveAlgaeCommand extends SequentialCommandGroup {
    
    public RemoveAlgaeCommand(ArmivatorCommands armivatorCommands, SwerveSubsystem drivebase, PoseNavigator poseNavigator) {
        addCommands(
            armivatorCommands.setArmivatorState(ElevatorStates.BOTTOM, ArmStates.L2),
            Commands.either(
                armivatorCommands.adjustArmSetpoint(-50).withTimeout(0.25),
                armivatorCommands.adjustArmSetpoint(-210).withTimeout(0.6),
                poseNavigator::isHighAlgae
            ),
            Commands.defer(
                () -> drivebase.driveToPose(poseNavigator.calculateClosestReefPose()),
                Set.of(drivebase)
            ),
            armivatorCommands.adjustArmSetpoint(230).withTimeout(0.6),
            drivebase.driveToDistance(-0.75)
        );
    }
}
