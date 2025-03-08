package frc.robot.commands.serializer;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotStates.Arm.ArmStates;
import frc.robot.Constants.RobotStates.Elevator.ElevatorStates;
import frc.robot.commands.armivator.KahChunkCommand;
import frc.robot.commands.armivator.SetArmivatorState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SerializerSubsystem;

public class PickupCoralCommand extends SequentialCommandGroup {
    
    public PickupCoralCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, SerializerSubsystem serializerSubsystem) {
        addRequirements(elevatorSubsystem, armSubsystem, serializerSubsystem);

        addCommands(
            Commands.parallel(
                serializerSubsystem.runSerializerCommand()
                    .until(serializerSubsystem::isCoralLoaded),
                new SetArmivatorState(elevatorSubsystem, armSubsystem, ElevatorStates.PICKUP, ArmStates.BOTTOM)
            ),
            new KahChunkCommand(elevatorSubsystem, armSubsystem)
        );
    }
}
