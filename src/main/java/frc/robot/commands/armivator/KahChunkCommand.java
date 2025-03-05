package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class KahChunkCommand extends SequentialCommandGroup {
    
    ElevatorSubsystem elevatorSubsystem;
    ArmSubsystem armSubsystem;

    public KahChunkCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;

        addCommands(
            Commands.parallel(
                elevatorSubsystem.goToStateCommand(ElevatorStates.PICKUP),
                armSubsystem.goToStateCommand(ArmStates.BOTTOM)
            ),
            elevatorSubsystem.goToStateCommand(ElevatorStates.BOTTOM),
            elevatorSubsystem.goToStateCommand(ElevatorStates.PICKUP)
        );
    }
}
