package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ScoreL2Command extends SequentialCommandGroup {

    public ScoreL2Command(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        ArmStates currentArmState = null;
        ArmStates targetArmState = null;
        ElevatorStates currentElevatorState = null;
        ElevatorStates targetElevatorState = null;
        
        boolean currentlyKachunked = currentArmState == ArmStates.BOTTOM && currentElevatorState == ElevatorStates.BOTTOM;
        boolean targetKachunked = targetArmState == ArmStates.BOTTOM && targetElevatorState == ElevatorStates.BOTTOM;

        if (
            (currentlyKachunked && targetArmState != ArmStates.BOTTOM) ||
            (targetKachunked && currentArmState != ArmStates.BOTTOM)
        ) {
            addCommands(elevatorSubsystem.GoToStateCommand(ElevatorStates.PICKUP));
        }

        addCommands(
            Commands.parallel(
                elevatorSubsystem.GoToStateCommand(ElevatorStates.PICKUP),
                armSubsystem.GoToStateCommand(ArmStates.L2),
                elevatorSubsystem.GoToStateCommand(ElevatorStates.BOTTOM)
            )
        );
    }
}