package frc.robot.commands.serializer;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotStates.Arm.ArmStates;
import frc.robot.Constants.RobotStates.Elevator.ElevatorStates;
import frc.robot.commands.armivator.ArmivatorCommands;
import frc.robot.commands.armivator.KahChunkCommand;
import frc.robot.subsystems.SerializerSubsystem;

public class PickupCoralCommand extends SequentialCommandGroup {
    
    public PickupCoralCommand(ArmivatorCommands armivatorCommands, SerializerSubsystem serializerSubsystem) {
        addCommands(
            Commands.parallel(
                serializerSubsystem.runSerializerCommand(),
                armivatorCommands.setArmivatorState(ElevatorStates.PICKUP, ArmStates.BOTTOM)
            ),
            new KahChunkCommand(armivatorCommands)
        );
    }
}
