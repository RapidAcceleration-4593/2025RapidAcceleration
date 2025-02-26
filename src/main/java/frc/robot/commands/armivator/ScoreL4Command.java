package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ScoreL4Command extends SequentialCommandGroup {

    public ScoreL4Command(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        
        ElevatorStates currentElevatorState = elevatorSubsystem.getCurrentElevatorState();

        if (
            currentElevatorState != ElevatorStates.PICKUP &&
            currentElevatorState != ElevatorStates.TOP
        ) {
            addCommands(elevatorSubsystem.GoToStateCommand(ElevatorStates.PICKUP));
        }

        addCommands(
            Commands.parallel(
                armSubsystem.GoToStateCommand(ArmStates.TOP),
                elevatorSubsystem.GoToStateCommand(ElevatorStates.TOP)
            )
        );
    }
}
