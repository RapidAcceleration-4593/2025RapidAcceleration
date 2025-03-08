package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.RobotStates.Arm.ArmDirections;
import frc.robot.Constants.RobotStates.Arm.ArmStates;
import frc.robot.Constants.RobotStates.Elevator.ElevatorStates;
import frc.robot.commands.arm.SetArmState;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetArmivatorState extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final ArmSubsystem armSubsystem;

    private final ElevatorStates targetElevatorState;
    private final ArmStates targetArmState;

    private final Command pureSequentialSequence;
    private final Command topToBottomSequence;
    private final Command bottomToTopSequence;
    private final Command pureParallelSequence;

    private Command selectedSequence;

    public SetArmivatorState(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, ElevatorStates elevatorState, ArmStates armState) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;

        this.targetElevatorState = elevatorState;
        this.targetArmState = armState;
        
        // Register commands (this way they only have to be created once)
        pureSequentialSequence = Commands.sequence(
            new SetElevatorState(elevatorSubsystem, ElevatorStates.PICKUP).withTimeout(1.25),
            new SetArmState(armSubsystem, targetArmState).withTimeout(0.8),
            new SetElevatorState(elevatorSubsystem, targetElevatorState).withTimeout(1.25)
        );

        topToBottomSequence = Commands.sequence(
            Commands.parallel(
                new SetElevatorState(elevatorSubsystem, ElevatorStates.PICKUP).withTimeout(1.25),
                new SetArmState(armSubsystem, targetArmState).withTimeout(0.8)
            ),
            new SetElevatorState(elevatorSubsystem, targetElevatorState).withTimeout(1.25)
        );

        bottomToTopSequence = Commands.sequence(
            new SetElevatorState(elevatorSubsystem, ElevatorStates.PICKUP).withTimeout(1.25),
            Commands.parallel(
                new SetElevatorState(elevatorSubsystem, targetElevatorState).withTimeout(1.25),
                new SetArmState(armSubsystem, targetArmState).withTimeout(0.8)
            )
        );

        pureParallelSequence = Commands.parallel(
            new SetElevatorState(elevatorSubsystem, targetElevatorState).withTimeout(1.25),
            new SetArmState(armSubsystem, targetArmState).withTimeout(0.8)
        );

        addRequirements(elevatorSubsystem, armSubsystem);
    }

    @Override
    public void initialize() {
        selectedSequence = determineMovementSequence();
        selectedSequence.initialize();
    }

    @Override
    public void execute() {
        selectedSequence.execute();
    }

    @Override
    public void end(boolean interrupted) {
        selectedSequence.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return selectedSequence.isFinished();
    }

    private Command determineMovementSequence() {
        boolean armUp = armSubsystem.isArmUp() == ArmDirections.UP;
        boolean elevatorUp = elevatorSubsystem.isElevatorUp();

        boolean targetArmUp = targetArmState != ArmStates.BOTTOM;
        boolean targetElevatorUp = targetElevatorState != ElevatorStates.BOTTOM;

        // This sequence is the most stable. Elevator moves to PICKUP, the arm goes to target, and then elevator moves to target.
        // No matter what position the armivator is in, this sequence will always work.
        if (armSubsystem.isArmUp() == ArmDirections.UNKNOWN ||          // If we don't know where the arm is, we'd best use the safest sequence possible
            !elevatorUp && !targetElevatorUp && armUp != targetArmUp) { // Elevator down and wants to stay down but switch arm position
            return pureSequentialSequence;
        }

        if (elevatorUp && armUp && !targetElevatorUp && !targetArmUp) {     // Elevator all the way up and wants to kah-chunk.
            return topToBottomSequence;
        }

        if (!elevatorUp && !armUp && targetElevatorUp && targetArmUp) {     // Elevator kah-chunked and wants to go all the way up.            
            return bottomToTopSequence;
        }

        // This sequence is least stable and is only used if there is no chance that the arm will hit the kahchunk block.
        return pureParallelSequence;
    }
}
