package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorTravelTime;
import frc.robot.Constants.RobotStates.ArmStates;
import frc.robot.Constants.RobotStates.ElevatorStates;

public class KahChunkCommand extends SequentialCommandGroup {

    public KahChunkCommand(ArmivatorCommands armivatorCommands) {
        addCommands(
            armivatorCommands.setArmivatorState(ElevatorStates.BOTTOM, ArmStates.BOTTOM),
            armivatorCommands.setElevatorState(ElevatorStates.PICKUP).withTimeout(ElevatorTravelTime.KAH_CHUNK)
        );
    }
}
