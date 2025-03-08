package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotStates.Arm.ArmStates;
import frc.robot.Constants.RobotStates.Elevator.ElevatorStates;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.SetArmState;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class KahChunkCommand extends SequentialCommandGroup {
    
    public KahChunkCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        addCommands(
            Commands.parallel(
                new SetElevatorState(elevatorSubsystem, ElevatorStates.PICKUP).withTimeout(ElevatorConstants.MAX_TRAVEL_TIME),
                new SetArmState(armSubsystem, ArmStates.BOTTOM).withTimeout(ArmConstants.MAX_TRAVEL_TIME)
            ),
            new SetElevatorState(elevatorSubsystem, ElevatorStates.BOTTOM).withTimeout(ElevatorConstants.MAX_TRAVEL_TIME),
            new SetElevatorState(elevatorSubsystem, ElevatorStates.PICKUP).withTimeout(ElevatorConstants.MAX_TRAVEL_TIME)
        );
    }
}
