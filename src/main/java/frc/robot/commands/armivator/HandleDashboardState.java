package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.RobotStates.ArmStates;
import frc.robot.Constants.RobotStates.ElevatorStates;
import frc.robot.subsystems.utils.PoseNavigator;

public class HandleDashboardState extends Command {

    private final ArmivatorCommands armivatorCommands;
    private final PoseNavigator poseNavigator;

    public HandleDashboardState(ArmivatorCommands armivatorCommands, PoseNavigator poseNavigator) {
        this.armivatorCommands = armivatorCommands;
        this.poseNavigator = poseNavigator;
    }

    @Override
    public void initialize() {
        Command selectedCommand = switch (poseNavigator.getTargetArmivatorState()) {
            case 1 -> armivatorCommands.setArmivatorState(ElevatorStates.BOTTOM, ArmStates.BOTTOM);
            case 2 -> armivatorCommands.setArmivatorState(ElevatorStates.BOTTOM, ArmStates.L2);
            case 3 -> armivatorCommands.setArmivatorState(ElevatorStates.BOTTOM, ArmStates.TOP);
            case 4 -> armivatorCommands.setArmivatorState(ElevatorStates.TOP, ArmStates.TOP);
            default -> Commands.none();
        };

        selectedCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
