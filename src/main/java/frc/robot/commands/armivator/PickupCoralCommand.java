package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotStates.ArmStates;
import frc.robot.Constants.RobotStates.ElevatorStates;
import frc.robot.Constants.RobotStates.IntakeStates;
import frc.robot.commands.intake.RunIntakeCommand;
import frc.robot.commands.intake.SetIntakeState;
import frc.robot.commands.serializer.RunSerializerCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SerializerSubsystem;

public class PickupCoralCommand extends SequentialCommandGroup {
    
    public PickupCoralCommand(ArmivatorCommands armivatorCommands, SerializerSubsystem serializerSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
            Commands.parallel(
                armivatorCommands.setArmivatorState(ElevatorStates.PICKUP, ArmStates.BOTTOM),
                new SetIntakeState(intakeSubsystem, IntakeStates.OUT)
            ),
            Commands.race(
                new RunIntakeCommand(intakeSubsystem, false),
                new RunSerializerCommand(serializerSubsystem, false, true)
            ),
            Commands.parallel(
                new SetIntakeState(intakeSubsystem, IntakeStates.IN),
                new KahChunkCommand(armivatorCommands)
            )
        );
    }
}
