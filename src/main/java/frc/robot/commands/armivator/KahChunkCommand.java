package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorTravelTime;
import frc.robot.Constants.RobotStates.Arm.ArmStates;
import frc.robot.Constants.RobotStates.Elevator.ElevatorStates;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class KahChunkCommand extends SequentialCommandGroup {

    public KahChunkCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        addCommands(
            new SetArmivatorState(elevatorSubsystem, armSubsystem, ElevatorStates.BOTTOM, ArmStates.BOTTOM),
            new SetElevatorState(elevatorSubsystem, ElevatorStates.PICKUP).withTimeout(ElevatorTravelTime.BOTTOM_TO_PICKUP)
        );
    }
}
