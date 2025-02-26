package frc.robot.commands.armivator;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ArmSubsystem.IsUpResult;

public class GoToPositionCommand extends SequentialCommandGroup {

    ElevatorSubsystem elevatorSubsystem;
    ArmSubsystem armSubsystem;


    public GoToPositionCommand(ArmStates armState, ElevatorStates elevatorState, ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;

        addCommands(
            elevatorSubsystem.GoToStateCommand(ElevatorStates.PICKUP),
            armSubsystem.GoToStateCommand(ArmStates.L2),
            elevatorSubsystem.GoToStateCommand(ElevatorStates.BOTTOM)
        );
    }

    private void handleKachunkCollision(ArmStates targetArm, ElevatorStates targetElevator) {
        var isArmUpResult = armSubsystem.isUp(); // Arm may be uncertain if it is up
        if (isArmUpResult == IsUpResult.UNSURE) {
            addCommands(elevatorSubsystem.GoToStateCommand(ElevatorStates.PICKUP)); // Best be careful, the arm isn't sure if it is up or down
            return;
        }

        boolean armUp = isArmUpResult == IsUpResult.UP;
        boolean elevatorUp = elevatorSubsystem.isUp();

        boolean targetArmUp = targetArm != ArmStates.BOTTOM;
        boolean targetElevatorUp = targetElevator != ElevatorStates.BOTTOM;

        boolean requireIntakeState = false;
                                                                                        // If any of these conditions are true, we need to go to INTAKE first
        requireIntakeState |= !elevatorUp && !armUp && targetElevatorUp && targetArmUp; // 1. Elevator kachunked, and wants to go all the way up
        requireIntakeState |= elevatorUp && armUp && !targetElevatorUp && !targetArmUp; // 2. The oppposite of 1.

        requireIntakeState |= !elevatorUp && !armUp && !targetElevatorUp && targetArmUp;// 3. Elevator kachunked, and wants to keep elevator down but arm out
        requireIntakeState |= !elevatorUp && armUp && !targetElevatorUp && !targetArmUp;// 4. Elevator down, arm up, and wants to go to kachunk.
        
        if (requireIntakeState) {
            addCommands(elevatorSubsystem.GoToStateCommand(ElevatorStates.PICKUP)); // Best be careful, the arm isn't sure if it is up or down
        }
    }
    
}