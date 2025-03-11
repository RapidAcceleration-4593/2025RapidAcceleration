package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ArmConstants.ArmTravelTime;
import frc.robot.Constants.ElevatorConstants.ElevatorTravelTime;
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

    private Command selectedSequence;

    public SetArmivatorState(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, ElevatorStates elevatorState, ArmStates armState) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;

        this.targetElevatorState = elevatorState;
        this.targetArmState = armState;

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

        Command moveToPickup = new SetElevatorState(elevatorSubsystem, ElevatorStates.PICKUP).withTimeout(ElevatorTravelTime.MAX_TRAVEL);
        Command moveArm = new SetArmState(armSubsystem, targetArmState).withTimeout(ArmTravelTime.MAX_TRAVEL);
        Command moveElevator = new SetElevatorState(elevatorSubsystem, targetElevatorState).withTimeout(ElevatorTravelTime.MAX_TRAVEL);

        // Most Stable Sequence. Elevator moves to PICKUP, Arm goes to target, and Elevator moves to target.
        // Sequence will always work, no matter the position of the Armivator.
        if (armSubsystem.isArmUp() == ArmDirections.UNKNOWN ||          // Unknown Arm Position; use the safest sequence possible.
            !elevatorUp && !targetElevatorUp && armUp != targetArmUp) { // Elevator at BOTTOM and target is BOTTOM; switch arm position.
            return Commands.sequence(moveToPickup, moveArm, moveElevator);
        }

        if (elevatorUp && !targetElevatorUp && armUp != targetArmUp) {  // Elevator at TOP and wants to go to BOTTOM.
            return Commands.sequence(Commands.parallel(moveToPickup, moveArm), moveElevator);
        }

        if (!elevatorUp && targetElevatorUp && armUp != targetArmUp) {  // Elevator at BOTTOM and wants to go to TOP.           
            return Commands.sequence(moveToPickup, Commands.parallel(moveElevator, moveArm));
        }

        // Least Stable Sequence. Only used if there is no chance that the arm will hit the BUMPER.
        return Commands.parallel(moveElevator, moveArm);
    }
}
