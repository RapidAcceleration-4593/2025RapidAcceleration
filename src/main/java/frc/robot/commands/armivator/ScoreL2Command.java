package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ScoreL2Command extends SequentialCommandGroup {

    public ScoreL2Command(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        addCommands(
            elevatorSubsystem.GoToStateCommand(ElevatorStates.PICKUP),
            armSubsystem.GoToStateCommand(ArmStates.L2),
            new SetElevatorState(elevatorSubsystem, ElevatorStates.BOTTOM)
        );
    }
}