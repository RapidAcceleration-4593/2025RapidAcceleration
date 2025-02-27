package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.Command;
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
            Commands.either(
                Commands.either(
                    null,
                    Commands.sequence(
                        elevatorSubsystem.GoToStateCommand(ElevatorStates.PICKUP),
                        elevatorSubsystem.GoToStateCommand(elevatorState),
                        armSubsystem.GoToStateCommand(armState)
                    ),
                    null
                ),
                Commands.parallel(
                    elevatorSubsystem.GoToStateCommand(elevatorState),
                    armSubsystem.GoToStateCommand(armState)
                ),
                this::willCollide
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

    /**
     * Can the arm and elevator move in parallel, or must they be moved sequentially?
     * Note that this method will return a nonsensical answer if willCollide returns false.
     * @return Whether the arm and elevator can move to the target in parallel.
     */
    private boolean parallelSupported() {
        boolean elevatorUp = elevatorSubsystem.isElevatorUp();
        boolean targetElevatorUp = targetElevator != ElevatorStates.BOTTOM;

        return elevatorUp || elevatorUp != targetElevatorUp;
    }

    private boolean isKahchunkToL4 () {
        boolean armUp = armSubsystem.isArmUp();
        boolean elevatorUp = elevatorSubsystem.isElevatorUp();

        boolean targetArmUp = targetArm != ArmStates.BOTTOM;
        boolean targetElevatorUp = targetElevator != ElevatorStates.BOTTOM;

        return !elevatorUp && !armUp && targetElevatorUp && targetArmUp;
    }

    private Command handleCollisionCommand() {
        return Commands.either(
            Commands.either(
                Commands.sequence(
                    elevatorSubsystem.GoToStateCommand(ElevatorStates.PICKUP),
                    Commands.parallel(
                        elevatorSubsystem.GoToStateCommand(targetElevator),
                        armSubsystem.GoToStateCommand(targetArm)
                    )
                ),
                Commands.sequence(
                    Commands.parallel(
                        elevatorSubsystem.GoToStateCommand(ElevatorStates.PICKUP),
                        armSubsystem.GoToStateCommand(targetArm)
                    ),
                    elevatorSubsystem.GoToStateCommand(targetElevator)
                ),
                this::isKahchunkToL4
            ),
            Commands.sequence(
                elevatorSubsystem.GoToStateCommand(ElevatorStates.PICKUP),
                elevatorSubsystem.GoToStateCommand(targetElevator),
                armSubsystem.GoToStateCommand(targetArm)
            ),
            this::parallelSupported
        );
    }
}