package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotStates.ArmStates;
import frc.robot.Constants.RobotStates.ElevatorStates;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SerializerSubsystem;

public class PickupCoralCommand extends SequentialCommandGroup {
    
    public PickupCoralCommand(ArmivatorCommands armivatorCommands, SerializerSubsystem serializerSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
            Commands.parallel(
                armivatorCommands.setArmivatorState(ElevatorStates.PICKUP, ArmStates.BOTTOM),
                intakeSubsystem.runIntakeCommand(),
                serializerSubsystem.runSerializerCommand()
            ),
            intakeSubsystem.stopIntakeCommand(),
            new KahChunkCommand(armivatorCommands)
        );
    }
}
