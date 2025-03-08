package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotStates.Arm.ArmStates;
import frc.robot.Constants.RobotStates.Elevator.ElevatorStates;
import frc.robot.commands.arm.ScoreCoralCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RemoveAlgaeCommand extends SequentialCommandGroup {
    
    public RemoveAlgaeCommand(SwerveSubsystem drivebase, ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, boolean isHighAlgae) {
        addCommands(
            Commands.either(
                new SetArmivatorState(elevatorSubsystem, armSubsystem, ElevatorStates.TOP, ArmStates.L2),
                new SetArmivatorState(elevatorSubsystem, armSubsystem, ElevatorStates.BOTTOM, ArmStates.TOP),
                () -> isHighAlgae
            ),
            Commands.parallel(
                new ScoreCoralCommand(armSubsystem).withTimeout(0.3),
                drivebase.driveBackward()
            )
        );
    }
}
