package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ScoreL2Command extends SequentialCommandGroup {

    public ScoreL2Command(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        
        ElevatorStates currentElevatorState = elevatorSubsystem.getCurrentElevatorState();
        ArmStates currentArmState = armSubsystem.getCurrentArmState();

        if (
            currentElevatorState != ElevatorStates.PICKUP &&
            currentElevatorState != ElevatorStates.TOP
        ) {
            addCommands(elevatorSubsystem.GoToStateCommand(ElevatorStates.PICKUP));
        }

        addCommands(
            Commands.parallel(
                armSubsystem.GoToStateCommand(ArmStates.L2),
                elevatorSubsystem.GoToStateCommand(ElevatorStates.BOTTOM)
            )
        );
    }
}