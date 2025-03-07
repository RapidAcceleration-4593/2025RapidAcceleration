package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotStates.Arm.ArmDirections;
import frc.robot.Constants.RobotStates.Arm.ArmStates;
import frc.robot.Constants.RobotStates.Elevator.ElevatorStates;
import frc.robot.commands.arm.SetArmState;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetArmivatorState extends SequentialCommandGroup {

    private final ElevatorSubsystem elevatorSubsystem;
    private final ArmSubsystem armSubsystem;

    private final ElevatorStates targetElevatorState;
    private final ArmStates targetArmState;

    public SetArmivatorState(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, ElevatorStates elevatorState, ArmStates armState) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;

        this.targetElevatorState = elevatorState;
        this.targetArmState = armState;
        
        addCommands(
            Commands.either(
                Commands.either(
                    handleCollisionCommand(),
                    Commands.parallel(
                        new SetElevatorState(elevatorSubsystem, elevatorState).withTimeout(1.25),
                        new SetArmState(armSubsystem, armState).withTimeout(0.8)
                    ),
                    this::willCollide
                ),
                Commands.sequence(
                    new SetElevatorState(elevatorSubsystem, ElevatorStates.PICKUP).withTimeout(1.25),
                    new SetArmState(armSubsystem, armState).withTimeout(0.8),
                    new SetElevatorState(elevatorSubsystem, elevatorState).withTimeout(1.25)
                ),
                () -> armSubsystem.isArmUp() != ArmDirections.UNKNOWN
            )
        );
    }

    private boolean willCollide() {
        boolean armUp = armSubsystem.isArmUp() == ArmDirections.UP;
        boolean elevatorUp = elevatorSubsystem.isElevatorUp();

        boolean targetArmUp = targetArmState != ArmStates.BOTTOM;
        boolean targetElevatorUp = targetElevatorState != ElevatorStates.BOTTOM;

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
        boolean targetElevatorUp = targetElevatorState != ElevatorStates.BOTTOM;

        return elevatorUp || elevatorUp != targetElevatorUp;
    }

    /**
     * Don't use this. It checks if the arm and elevator are currently down and are both moving to up.
     * @return
     */
    private boolean isKahchunkToRaised () {
        boolean armUp = armSubsystem.isArmUp() == ArmDirections.UP;
        boolean elevatorUp = elevatorSubsystem.isElevatorUp();

        boolean targetArmUp = targetArmState != ArmStates.BOTTOM;
        boolean targetElevatorUp = targetElevatorState != ElevatorStates.BOTTOM;

        return !elevatorUp && !armUp && targetElevatorUp && targetArmUp;
    }

    /**
     * This command handles the nitty gritty details of what to do when the armivator has to avoid hitting the kahchunk block and bumper.
     * @return The Command you call, when all is lost.
     */
    private Command handleCollisionCommand() {
        return Commands.either(
            Commands.either(
                Commands.sequence(
                    new SetElevatorState(elevatorSubsystem, ElevatorStates.PICKUP).withTimeout(1.25),
                    Commands.parallel(
                        new SetElevatorState(elevatorSubsystem, targetElevatorState).withTimeout(1.25),
                        new SetArmState(armSubsystem, targetArmState).withTimeout(0.8)
                    )
                ),
                Commands.sequence(
                    Commands.parallel(
                        new SetElevatorState(elevatorSubsystem, ElevatorStates.PICKUP).withTimeout(1.25),
                        new SetArmState(armSubsystem, targetArmState).withTimeout(0.8)
                    ),
                    new SetElevatorState(elevatorSubsystem, targetElevatorState).withTimeout(1.25)
                ),
                this::isKahchunkToRaised
            ),
            Commands.sequence(
                new SetElevatorState(elevatorSubsystem, ElevatorStates.PICKUP).withTimeout(1.25),
                new SetArmState(armSubsystem, targetArmState).withTimeout(0.8),
                new SetElevatorState(elevatorSubsystem, targetElevatorState).withTimeout(1.25)
            ),
            this::parallelSupported
        );
    }
}