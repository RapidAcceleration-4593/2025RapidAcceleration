package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorTravelTime;
import frc.robot.Constants.RobotStates.ArmStates;
import frc.robot.Constants.RobotStates.ElevatorStates;
import frc.robot.Constants.RobotStates.IntakeStates;
import frc.robot.commands.armivator.base.ArmivatorCommands;
import frc.robot.commands.intake.RunIntakeCommand;
import frc.robot.commands.intake.base.SetIntakeState;
import frc.robot.commands.serializer.RunSerializerCommand;
import frc.robot.subsystems.IntakeDeploySubsystem;
import frc.robot.subsystems.IntakeFeederSubsystem;
import frc.robot.subsystems.SerializerSubsystem;

public class PickupCoralCommand extends SequentialCommandGroup {
    
    public PickupCoralCommand(ArmivatorCommands armivatorCommands, SerializerSubsystem serializerSubsystem, IntakeDeploySubsystem deploySubsystem, IntakeFeederSubsystem feederSubsystem) {
        addCommands(
            Commands.parallel(
                armivatorCommands.setArmivatorState(ElevatorStates.PICKUP, ArmStates.BOTTOM),
                new SetIntakeState(deploySubsystem, IntakeStates.OUT)
            ),
            Commands.race(
                new RunIntakeCommand(feederSubsystem, false),
                new RunSerializerCommand(serializerSubsystem, false, true)
            ),
            Commands.parallel(
                new SetIntakeState(deploySubsystem, IntakeStates.L1).withTimeout(1.5),
                Commands.sequence(
                    armivatorCommands.setArmivatorState(ElevatorStates.BOTTOM, ArmStates.BOTTOM),
                    armivatorCommands.setElevatorState(ElevatorStates.PICKUP).withTimeout(ElevatorTravelTime.KAH_CHUNK)
                )
            )
        );
    }
}
