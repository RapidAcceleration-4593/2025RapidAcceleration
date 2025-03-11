package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotStates.Elevator.ElevatorStates;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class KahChunkCommand extends SequentialCommandGroup {

    public KahChunkCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        addCommands(
            // Commands.parallel(
            //     new SetElevatorState(elevatorSubsystem, ElevatorStates.PICKUP).withTimeout(ElevatorConstants.MAX_TRAVEL_TIME),
            //     new SetArmState(armSubsystem, ArmStates.BOTTOM).withTimeout(ArmConstants.MAX_TRAVEL_TIME)
            // ),
            new SetElevatorState(elevatorSubsystem, ElevatorStates.BOTTOM).withTimeout(0.8),
            new SetElevatorState(elevatorSubsystem, ElevatorStates.PICKUP).withTimeout(0.8)
        );
    }
}
