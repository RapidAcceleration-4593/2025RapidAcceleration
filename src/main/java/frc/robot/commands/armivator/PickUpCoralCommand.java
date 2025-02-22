package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class PickUpCoralCommand extends SequentialCommandGroup {

    public PickUpCoralCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        addCommands(
            armSubsystem.GoToStateCommand(ArmStates.BOTTOM),
            elevatorSubsystem.GoToStateCommand(ElevatorStates.BOTTOM)
        );
    }
}
