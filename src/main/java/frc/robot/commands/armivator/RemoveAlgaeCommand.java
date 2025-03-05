package frc.robot.commands.armivator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RemoveAlgaeCommand extends SequentialCommandGroup {
    
    public RemoveAlgaeCommand(SwerveSubsystem drivebase, ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, boolean isHighAlgae) {

        addCommands(
            Commands.either(
                new GoToPositionCommand(elevatorSubsystem, armSubsystem, ElevatorStates.TOP, ArmStates.L2),
                new GoToPositionCommand(elevatorSubsystem, armSubsystem, ElevatorStates.BOTTOM, ArmStates.TOP),
                () -> isHighAlgae
            ),
            Commands.parallel(
                Commands.race(
                    armSubsystem.scoreCoralCommand(),
                    Commands.waitSeconds(0.5)
                ),
                drivebase.driveToDistanceCommand(-.5, 1)
            )
        );
    }
}
