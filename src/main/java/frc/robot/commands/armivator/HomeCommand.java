package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.commands.arm.SetArmState;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class HomeCommand extends ParallelCommandGroup {

    public HomeCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        addCommands(
            new SetElevatorState(elevatorSubsystem, ElevatorStates.PICKUP),
            new SetArmState(armSubsystem, ArmStates.BOTTOM)
        );
    }
}
