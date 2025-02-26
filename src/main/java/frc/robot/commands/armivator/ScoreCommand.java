package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ScoreCommand extends SequentialCommandGroup {
    
    public ScoreCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, ElevatorStates targetElevatorState, ArmStates targetArmState) {
        
        ElevatorStates previousElevatorState = elevatorSubsystem.getCurrentElevatorState();
        ArmStates previousArmState = armSubsystem.getCurrentArmState();

        // Handle direct L2 <-> L3 transitions (only move the arm)
        if ((previousArmState == ArmStates.L2 && targetArmState == ArmStates.TOP) ||
            (previousArmState == ArmStates.TOP && targetArmState == ArmStates.L2)) {
            addCommands(armSubsystem.GoToStateCommand(targetArmState));
            return;
        }

        // BOTTOM → L2 or L3: Elevator to PICKUP first
        if (previousElevatorState == ElevatorStates.BOTTOM && (targetArmState == ArmStates.L2 || targetArmState == ArmStates.TOP)) {
            addCommands(elevatorSubsystem.GoToStateCommand(ElevatorStates.PICKUP));
        }

        // L4 → BOTTOM: Elevator to PICKUP first, then lower both
        if (previousElevatorState == ElevatorStates.TOP && targetElevatorState == ElevatorStates.BOTTOM) {
            addCommands(elevatorSubsystem.GoToStateCommand(ElevatorStates.PICKUP));
            addCommands(Commands.parallel(
                elevatorSubsystem.GoToStateCommand(ElevatorStates.BOTTOM),
                armSubsystem.GoToStateCommand(ArmStates.BOTTOM)
            ));
            return;
        }

        // Moving to L4: Move elevator to TOP before arm moves
        if (targetElevatorState == ElevatorStates.TOP) {
            addCommands(Commands.parallel(
                elevatorSubsystem.GoToStateCommand(ElevatorStates.TOP),
                armSubsystem.GoToStateCommand(ArmStates.TOP)
            ));
            return;
        }

        // Moving to BOTTOM: Elevator to PICKUP first if needed, then lower together
        if (targetArmState == ArmStates.BOTTOM) {
            addCommands(Commands.parallel(
                elevatorSubsystem.GoToStateCommand(ElevatorStates.PICKUP),
                armSubsystem.GoToStateCommand(ArmStates.BOTTOM)
            ));
            addCommands(elevatorSubsystem.GoToStateCommand(ElevatorStates.BOTTOM));
            return;
        }

        // L4 → L2: Move both down together
        if (previousElevatorState == ElevatorStates.TOP && targetArmState == ArmStates.L2) {
            addCommands(Commands.parallel(
                elevatorSubsystem.GoToStateCommand(ElevatorStates.BOTTOM),
                armSubsystem.GoToStateCommand(ArmStates.L2)
            ));
            return;
        }

        // Default: Move arm and elevator in parallel
        addCommands(Commands.parallel(
            armSubsystem.GoToStateCommand(targetArmState),
            elevatorSubsystem.GoToStateCommand(targetElevatorState)
        ));
    }
}
