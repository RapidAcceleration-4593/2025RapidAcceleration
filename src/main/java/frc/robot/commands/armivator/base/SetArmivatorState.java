package frc.robot.commands.armivator.base;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ArmConstants.ArmTravelTime;
import frc.robot.Constants.ElevatorConstants.ElevatorTravelTime;
import frc.robot.Constants.RobotStates.ArmStates;
import frc.robot.Constants.RobotStates.ElevatorStates;
import frc.robot.commands.arm.base.SetArmState;
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
        ElevatorStates currentElevatorState = elevatorSubsystem.getCurrentState();
        ArmStates currentArmState = armSubsystem.getCurrentState();

        boolean isElevatorUp = currentElevatorState != ElevatorStates.BOTTOM;
        boolean isArmUp = currentArmState != ArmStates.BOTTOM;

        boolean isTargetElevatorUp = targetElevatorState != ElevatorStates.BOTTOM;
        boolean isTargetArmUp = targetArmState != ArmStates.BOTTOM;

        Command moveToPickup = new SetElevatorState(elevatorSubsystem, ElevatorStates.PICKUP).withTimeout(ElevatorTravelTime.MAX_TRAVEL);
        Command moveElevator = new SetElevatorState(elevatorSubsystem, targetElevatorState).withTimeout(ElevatorTravelTime.MAX_TRAVEL);
        Command moveArm = new SetArmState(armSubsystem, targetArmState).withTimeout(ArmTravelTime.MAX_TRAVEL);

        // Movement between BOTTOM and PICKUP. Only move the Elevator.
        if (isElevatorUp != isTargetElevatorUp && currentArmState == targetArmState) {
            return moveElevator;
        }

        // Movement between L2 and L3. Only move the Arm.
        if (!isElevatorUp && !isTargetElevatorUp && isArmUp && isTargetArmUp && currentArmState != targetArmState) {
            return moveArm;
        }

        // Most Stable Sequence. Elevator moves to PICKUP, Arm goes to target, and Elevator moves to target.
        if (!isElevatorUp && !isTargetElevatorUp && isArmUp != isTargetArmUp) { 
            return Commands.sequence(moveToPickup, moveArm, moveElevator);
        }

        // Elevator moves to PICKUP and Arm goes to target, then Elevator goes to target.
        if (isElevatorUp && !isTargetElevatorUp && isArmUp != isTargetArmUp) {  
            return Commands.sequence(Commands.parallel(moveToPickup, moveArm), moveElevator);
        }

        // Elevator moves to PICKUP, then Elevator goes to target and Arm goes to target.
        if (!isElevatorUp && isTargetElevatorUp && !isArmUp && isTargetArmUp) {          
            return Commands.sequence(moveToPickup, Commands.parallel(moveElevator, moveArm));
        }

        // Least Stable Sequence. Only used if there is no chance that the arm will hit the BUMPER.
        return Commands.parallel(moveElevator, moveArm);
    }
}