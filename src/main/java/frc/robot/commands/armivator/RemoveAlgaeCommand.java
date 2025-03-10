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
import frc.robot.commands.arm.ScoreCoralCommand;

public class RemoveAlgaeCommand extends SequentialCommandGroup {
    
    public RemoveAlgaeCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, SwerveSubsystem drivebase, PoseNavigator poseNavigator) {
        addCommands(
            Commands.either(
                new SetArmivatorState(elevatorSubsystem, armSubsystem, ElevatorStates.TOP, ArmStates.L2),
                new SetArmivatorState(elevatorSubsystem, armSubsystem, ElevatorStates.BOTTOM, ArmStates.TOP),
                poseNavigator::isHighAlgae
            ),
            Commands.defer(
                () -> drivebase.driveToPose(poseNavigator.calculateClosestReefPose()),
                Set.of(drivebase)
            ),
            Commands.parallel(
                new ScoreCoralCommand(armSubsystem).withTimeout(0.3),
                drivebase.driveToDistance(-0.75).withTimeout(1.0)
            )
        );
    }
}
