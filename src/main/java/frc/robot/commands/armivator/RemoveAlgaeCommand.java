package frc.robot.commands.armivator;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotStates.Arm.ArmStates;
import frc.robot.Constants.RobotStates.Elevator.ElevatorStates;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.PoseNavigator;
import frc.robot.commands.arm.AdjustArmCommand;

public class RemoveAlgaeCommand extends SequentialCommandGroup {
    
    public RemoveAlgaeCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, SwerveSubsystem drivebase, PoseNavigator poseNavigator) {
        addCommands(
            new SetArmivatorState(elevatorSubsystem, armSubsystem, ElevatorStates.BOTTOM, ArmStates.L2),
            Commands.either(
                new AdjustArmCommand(armSubsystem, -50).withTimeout(0.4),
                new AdjustArmCommand(armSubsystem, -210).withTimeout(0.75),
                poseNavigator::isHighAlgae
            ),
            Commands.defer(
                () -> drivebase.driveToPose(poseNavigator.calculateClosestReefPose()),
                Set.of(drivebase)
            ),
            new AdjustArmCommand(armSubsystem, 200).withTimeout(0.4),
            drivebase.driveToDistance(-0.75)
        );
    }
}
