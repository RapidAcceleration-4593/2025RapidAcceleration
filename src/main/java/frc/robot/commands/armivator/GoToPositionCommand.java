package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class GoToPositionCommand extends SequentialCommandGroup {

    ElevatorSubsystem elevatorSubsystem;
    ArmSubsystem armSubsystem;

    ElevatorStates targetElevator;
    ArmStates targetArm;


    public GoToPositionCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, ElevatorStates elevatorState, ArmStates armState) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;

        this.targetElevator = elevatorState;
        this.targetArm = armState;
        
        addCommands(
            elevatorSubsystem.GoToStateCommand(ElevatorStates.PICKUP).onlyIf(this::willCollide),
            Commands.parallel(
                elevatorSubsystem.GoToStateCommand(elevatorState),
                armSubsystem.GoToStateCommand(armState)
            )
        );
    }

    private boolean willCollide() {
        // var isArmUpResult = armSubsystem.isArmUp(); // Arm may be uncertain if it is up.
        // if (isArmUpResult == ArmEncoderStates.UNKNOWN)
        //     return true;

        // boolean armUp = isArmUpResult == ArmEncoderStates.UP;
        boolean armUp = armSubsystem.isArmUp();
        boolean elevatorUp = elevatorSubsystem.isElevatorUp();

        boolean targetArmUp = targetArm != ArmStates.BOTTOM;
        boolean targetElevatorUp = targetElevator != ElevatorStates.BOTTOM;

        boolean requirePickupState = false;
                                                                                         // If any of these conditions are true, we need to go to INTAKE first.
        requirePickupState |= !elevatorUp && !armUp && targetElevatorUp && targetArmUp;  // 1. Elevator kah-chunked and wants to go all the way up.
        requirePickupState |= elevatorUp && armUp && !targetElevatorUp && !targetArmUp;  // 2. The opposite of 1.

        requirePickupState |= !elevatorUp && !armUp && !targetElevatorUp && targetArmUp; // 3. Elevator kah-chunked and wants to keep elevator down but arm out.
        requirePickupState |= !elevatorUp && armUp && !targetElevatorUp && !targetArmUp; // 4. Elevator down, arm up, and wants to go to kah-chunk.
        
        return requirePickupState;
    }
}