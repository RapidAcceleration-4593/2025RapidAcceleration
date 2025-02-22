package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.commands.arm.SetArmState;
import frc.robot.commands.elevator.ControlElevatorState;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ScoreL4Command extends SequentialCommandGroup {

    public ScoreL4Command(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        addCommands(
            new FunctionalCommand(
                () -> new SetElevatorState(elevatorSubsystem, ElevatorStates.PICKUP),
                () -> new ControlElevatorState(elevatorSubsystem, true),
                interrupted -> elevatorSubsystem.stopMotor(),
                () -> elevatorSubsystem.atSetpoint(),
                elevatorSubsystem
            ),
            new SetElevatorState(elevatorSubsystem, ElevatorStates.TOP),
            new SetArmState(armSubsystem, ArmStates.TOP)
        );
    }
}
