package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.commands.arm.ControlArmState;
import frc.robot.commands.arm.SetArmState;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class PickUpCoralCommand extends SequentialCommandGroup {

    public PickUpCoralCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        addCommands(
            new FunctionalCommand(
                () -> new SetArmState(armSubsystem, ArmStates.BOTTOM),
                () -> new ControlArmState(armSubsystem, true),
                interrupted -> armSubsystem.stopMotor(),
                () -> armSubsystem.atSetpoint(),
                armSubsystem
            ),
            new SetElevatorState(elevatorSubsystem, ElevatorStates.BOTTOM)
        );
    }
}
